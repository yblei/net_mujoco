"""
MuJoCo Web Viewer - Passive Viewer Interface

This module provides a launch_passive interface similar to mujoco.viewer.launch_passive,
but renders in a web browser instead of a native window.

The physics simulation runs in Python, and the browser is used only for visualization.
"""

import asyncio
import aiohttp
from pathlib import Path
import time
import mujoco
import tempfile
import shutil
import xml.etree.ElementTree as ET
import os
import zipfile
import base64
import io
import websockets
import json
import threading
from aiohttp import web


class PassiveWebViewer:
    """
    A passive web viewer for MuJoCo that mimics the mujoco.viewer.launch_passive interface.
    
    The viewer operates in a separate thread and does not block user code.
    Physics must be stepped by the user, and sync() must be called to update the visualization.
    """
    
    # Shared server state across all viewer instances
    _server_thread = None
    _server_started = False
    _clients = set()
    _current_viewer = None
    _cached_model_data = None
    
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 server_url: str = "http://localhost:9001",
                 model_path: str = None,
                 start_servers: bool = True):
        """
        Initialize the passive web viewer.

        Args:
            model: MuJoCo model instance
            data: MuJoCo data instance
            server_url: URL of the WebSocket server (ignored if start_servers=True)
            model_path: Original XML file path (for models with assets)
            start_servers: If True, starts WebSocket/HTTP servers automatically
        """
        self.model = model
        self.data = data
        self.server_url = server_url
        self.model_source = model_path
        self._running = True
        self._initial_sync_done = False
        self._last_sync_time = 0
        self._sync_interval = 1.0 / 60.0  # 60 FPS max
        self._start_servers = start_servers
        
        # Start servers if requested and not already running
        if start_servers and not PassiveWebViewer._server_started:
            self._start_background_servers()
        
        # Register as current viewer
        PassiveWebViewer._current_viewer = self
        
    async def _send_update(self, state_data: dict):
        """Send state update to the browser via HTTP API."""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.server_url}/update_state",
                    json=state_data,
                    headers={'Content-Type': 'application/json'},
                    timeout=aiohttp.ClientTimeout(total=1.0)
                ) as response:
                    return response.status == 200
        except Exception as e:
            # Silently fail - viewer might not be connected
            return False
    
    def sync(self, state_only: bool = True):
        """
        Synchronize the viewer with the current physics state.

        This should be called after stepping the physics to update
        the visualization.

        Args:
            state_only: If True, only sync state data.
                       If False, sync everything including model changes.
        """
        if not self._running:
            return

        # Throttle updates to max frame rate FIRST
        current_time = time.time()
        if current_time - self._last_sync_time < self._sync_interval:
            return
        self._last_sync_time = current_time

        # On first sync, send the full model
        if not self._initial_sync_done:
            self._send_initial_model()
            self._initial_sync_done = True
            return

        # Prepare state data to send
        state_data = {
            'type': 'state_update',
            'time': float(self.data.time),
            'qpos': self.data.qpos.tolist(),
            'qvel': self.data.qvel.tolist() if not state_only else None,
        }

        # Send synchronously
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._send_update(state_data))
            loop.close()
        except Exception:
            pass  # Silently fail if viewer not connected
    
    def _prepare_model_files_xml(self):
        """
        Prepare model by creating a zip containing XML and all mesh files.
        Uses mj_saveLastXML to resolve all includes and flatten the model.
        Returns base64-encoded zip file.
        """
        # Save the compiled model to XML (flattens all includes)
        temp_xml = Path(tempfile.mktemp(suffix='.xml'))
        mujoco.mj_saveLastXML(str(temp_xml), self.model)
        #raise NotImplementedError("the above fails if we create the whole thing from mjspec. Need to fix this tomorrow ...")
        
        # Parse to find mesh files
        tree = ET.parse(str(temp_xml))
        root = tree.getroot()
        
        # Get original XML directory for finding mesh files
        xml_dir = None
        print(f"   model.modelfiledir: {getattr(self.model, 'modelfiledir', 'N/A')}")
        print(f"   self.model_path: {self.model_source}")
        
        if hasattr(self.model, 'modelfiledir') and self.model.modelfiledir:
            xml_dir = Path(self.model.modelfiledir)
        elif self.model_source and Path(self.model_source).exists():
            xml_dir = Path(self.model_source).resolve().parent
        
        print(f"   XML directory: {xml_dir}")
        
        # Get meshdir from compiler if specified
        meshdir = ''
        compiler = root.find('compiler')
        if compiler is not None and 'meshdir' in compiler.attrib:
            meshdir = compiler.attrib['meshdir']
        
        # Collect all mesh file paths
        mesh_paths = {}
        mesh_elements = root.findall('.//mesh[@file]')
        print(f"   Found {len(mesh_elements)} mesh references in XML")
        
        for mesh in mesh_elements:
            mesh_file = mesh.attrib['file']
            
            # Try to find the mesh file
            full_path = None
            if xml_dir:
                if meshdir and not mesh_file.startswith(('/', '../')):
                    full_path = xml_dir / meshdir / mesh_file
                else:
                    full_path = xml_dir / mesh_file
                
                if full_path and full_path.exists():
                    # Map flattened name to actual path
                    mesh_paths[Path(mesh_file).name] = full_path
                    print(f"     ✓ Found: {mesh_file}")
        
        # Remove meshdir and adjust all paths to flat structure
        if compiler is not None and 'meshdir' in compiler.attrib:
            del compiler.attrib['meshdir']
        
        for mesh in root.findall('.//mesh[@file]'):
            orig_file = mesh.attrib['file']
            mesh.attrib['file'] = Path(orig_file).name
        
        # Generate final XML string with adjusted paths
        xml_content = ET.tostring(root, encoding='unicode')
        
        # Create zip file in memory
        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zf:
            # Add XML file
            zf.writestr('model.xml', xml_content)
            
            # Add all mesh files
            for mesh_name, mesh_path in mesh_paths.items():
                with open(mesh_path, 'rb') as f:
                    zf.writestr(mesh_name, f.read())
        
        # Base64 encode the zip
        zip_buffer.seek(0)
        zip_base64 = base64.b64encode(zip_buffer.read()).decode('utf-8')
        
        # Clean up temp file
        temp_xml.unlink()
        
        print(f"   Created zip with XML and {len(mesh_paths)} mesh files")
        print(f"   Zip size: {len(zip_base64)} bytes (base64)")
        
        return zip_base64
    
    def _prepare_model_files_mjspec(self):
        """
        Prepare model from mujoco.MjSpec by creating a zip containing XML and all mesh files.
        Returns base64-encoded zip file.
        """
        # Create temporary directory to save model files

        assert isinstance(self.model_source, mujoco.MjSpec)
        temp_dir = Path(tempfile.mkdtemp())
        
        try:
            # Save MjSpec to XML and assets
            xml_path = temp_dir / 'model.xml'
            self.model_source.to_file(str(xml_path))
            
            # Parse XML to find mesh files
            tree = ET.parse(str(xml_path))
            root = tree.getroot()
            
            # Collect all mesh file paths
            mesh_paths = {}
            mesh_elements = root.findall('.//mesh[@file]')
            print(f"   Found {len(mesh_elements)} mesh references in XML")

            asset_dir = Path(self.model_source.modelfiledir) / Path(self.model_source.meshdir)

            for mesh in mesh_elements:
                mesh_file = mesh.attrib['file']
                full_path = asset_dir / mesh_file
                
                if full_path.exists():
                    mesh_paths[Path(mesh_file).name] = full_path
                    print(f"     ✓ Found: {mesh_file}")
            
            # Remove meshdir and adjust all paths to flat structure
            compiler = root.find('compiler')
            if compiler is not None and 'meshdir' in compiler.attrib:
                del compiler.attrib['meshdir']
            
            for mesh in root.findall('.//mesh[@file]'):
                orig_file = mesh.attrib['file']
                mesh.attrib['file'] = Path(orig_file).name
            
            # Generate final XML string with adjusted paths
            xml_content = ET.tostring(root, encoding='unicode')
            
            # Create zip file in memory
            zip_buffer = io.BytesIO()
            with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zf:
                # Add XML file
                zf.writestr('model.xml', xml_content)
                
                # Add all mesh files
                for mesh_name, mesh_path in mesh_paths.items():
                    with open(mesh_path, 'rb') as f:
                        zf.writestr(mesh_name, f.read())
            
            # Base64 encode the zip
            zip_buffer.seek(0)
            zip_base64 = base64.b64encode(zip_buffer.read()).decode('utf-8')
            
            print(f"   Created zip with XML and {len(mesh_paths)} mesh files")
            print(f"   Zip size: {len(zip_base64)} bytes (base64)")

        except Exception as e:
            raise e
        
        return zip_base64

    def _send_initial_model(self):
        """Send the initial model as a zip file to the browser."""
        print(f"📤 Preparing model package...")
        
        # Create zip package with XML and all meshes
        if type(self.model_source) is str or self.model_source is None:
            zip_data = self._prepare_model_files_xml()

        elif isinstance(self.model_source, mujoco.MjSpec):
            zip_data = self._prepare_model_files_mjspec()

        else:
            raise ValueError("model_source must be a string path or mujoco.MjSpec")
        
        model_data = {
            'type': 'model_zip',
            'zip_data': zip_data
        }
        # Cache the model data for new browser connections
        PassiveWebViewer._cached_model_data = model_data
        
        async def send_model():
            try:
                print(f"   Posting to {self.server_url}/send_model...")
                async with aiohttp.ClientSession() as session:
                    async with session.post(
                        f"{self.server_url}/send_model",
                        json=model_data,
                        headers={'Content-Type': 'application/json'}
                    ) as response:
                        success = response.status == 200
                        msg = 'success' if success else 'failed'
                        print(f"   Server response: {response.status} ({msg})")
                        return success
            except Exception as e:
                print(f"   ❌ Error sending model: {e}")
                return False
        
        # Create new event loop to avoid conflicts with server thread
        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(send_model())
        finally:
            loop.close()
    
    def _start_background_servers(self):
        """Start WebSocket and HTTP servers in background thread."""
        def run_servers():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._run_servers())
        
        PassiveWebViewer._server_thread = threading.Thread(
            target=run_servers, daemon=True
        )
        PassiveWebViewer._server_thread.start()
        PassiveWebViewer._server_started = True
        print("=" * 60)
        print("MuJoCo Web Viewer - Servers Started")
        print("=" * 60)
        print("WebSocket: ws://localhost:9000")
        print("HTTP API:  http://localhost:9001")
        print("Open browser at: http://yblei.github.io/net_mujoco/")
        print("=" * 60)
    
    async def _run_servers(self):
        """Run WebSocket and HTTP servers."""
        # WebSocket handler
        async def ws_handler(websocket):
            PassiveWebViewer._clients.add(websocket)
            print(f"✓ Browser connected ({len(PassiveWebViewer._clients)} total)")
            
            # Send cached model to new client if available
            if PassiveWebViewer._cached_model_data:
                try:
                    message = json.dumps(PassiveWebViewer._cached_model_data)
                    await websocket.send(message)
                    print(f"   Sent cached model to new client")
                except Exception as e:
                    print(f"   Failed to send cached model: {e}")
            
            try:
                async for _ in websocket:
                    pass
            finally:
                PassiveWebViewer._clients.remove(websocket)
                print(f"✗ Browser disconnected ({len(PassiveWebViewer._clients)} total)")
        
        # HTTP handlers
        async def http_send_model(request):
            try:
                data = await request.json()
                if data.get('type') == 'model_zip':
                    message = json.dumps(data)
                    if PassiveWebViewer._clients:
                        await asyncio.gather(
                            *[c.send(message) for c in PassiveWebViewer._clients],
                            return_exceptions=True
                        )
                    return web.json_response({
                        'message': 'Sent',
                        'clients_count': len(PassiveWebViewer._clients)
                    })
                return web.json_response({'error': 'Invalid format'}, status=400)
            except Exception as e:
                return web.json_response({'error': str(e)}, status=500)
        
        async def http_update_state(request):
            try:
                state_data = await request.json()
                if PassiveWebViewer._clients:
                    message = json.dumps(state_data)
                    await asyncio.gather(
                        *[c.send(message) for c in PassiveWebViewer._clients],
                        return_exceptions=True
                    )
                return web.json_response({
                    'message': 'Updated',
                    'clients_count': len(PassiveWebViewer._clients)
                })
            except Exception as e:
                return web.json_response({'error': str(e)}, status=500)
        
        # Start WebSocket server
        ws_server = await websockets.serve(ws_handler, "localhost", 9000)
        
        # Start HTTP server
        app = web.Application(client_max_size=20*1024*1024)
        app.router.add_post('/send_model', http_send_model)
        app.router.add_post('/update_state', http_update_state)
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, 'localhost', 9001)
        await site.start()
        
        # Keep running
        await asyncio.Future()
    
    def close(self):
        """Close the viewer."""
        self._running = False
        if PassiveWebViewer._current_viewer == self:
            PassiveWebViewer._current_viewer = None
    
    def is_running(self) -> bool:
        """Check if the viewer is still running."""
        return self._running
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def launch_passive(model: mujoco.MjModel,
                   data: mujoco.MjData,
                   model_source: mujoco.MjSpec | str = None,
                   start_servers: bool = True,
                   open_browser: bool = True) -> PassiveWebViewer:
    """
    Launch a passive web viewer for MuJoCo.

    Similar to mujoco.viewer.launch_passive, but renders in a web browser.
    Automatically starts WebSocket and HTTP servers on first call.
    The user is responsible for stepping physics and calling sync().

    Args:
        model: MuJoCo model instance
        data: MuJoCo data instance
        model_source: Original XML file path (for models with mesh assets) ---------------------------------------------------------------------------------------------------------
        start_servers: If True (default), auto-starts servers. Set False
                      if you're running external websocket_server.py
        open_browser: If True (default), opens the web viewer in the default browser

    Returns:
        PassiveWebViewer instance that can be used as a context manager

    Example:
        >>> m = mujoco.MjModel.from_xml_path('model.xml')
        >>> d = mujoco.MjData(m)
        >>>
        >>> # Servers start automatically on first call
        >>> with launch_passive(m, d, model_path='model.xml') as viewer:
        ...     while viewer.is_running():
        ...         mujoco.mj_step(m, d)
        ...         viewer.sync()
        ...         time.sleep(m.opt.timestep)
    """
    viewer = PassiveWebViewer(
        model, data,
        server_url="http://localhost:9001",
        model_path=model_source,
        start_servers=start_servers
    )
    if open_browser:
        import webbrowser
        webbrowser.open("http://yblei.github.io/net_mujoco/")
    return viewer
