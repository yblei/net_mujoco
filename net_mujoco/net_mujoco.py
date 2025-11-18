import mujoco
import mujoco.viewer
import time
import os
from Xlib import X, display as xdisplay
import subprocess



class NetMujocoViewer:
    """Context manager for running a NetMujoco viewer session.

    This context manager sets up a NetMujoco viewer session, yielding
    a viewer instance and connection URL. It ensures proper cleanup
    of resources upon exit.

    Yields:
        tuple: A tuple containing the viewer instance and connection URL.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        display: str = ":100",
    ):
        self.model = model
        self.data = data
        self.display = display
        self.viewer = None
        self.display_backup = None 

    def __enter__(self):

        # check, if xpra is running. If not "No xpra sessions found" in output, raise error
        result = subprocess.run(
            ["xpra", "list"],
            check=True,
            capture_output=True,
            text=True,
        )
        if "No xpra sessions" in result.stdout:
            raise RuntimeError("Xpra server is not running. Start with python -m net_mujoco.server")

        self.display_backup = os.environ.get("DISPLAY")
        os.environ["DISPLAY"] = ":100"
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Fullscreen the window on :100
        time.sleep(0.5)  # Wait for window to appear
        try:
            
            disp = xdisplay.Display(":100")  # Connect to the Xpra display
            root = disp.screen().root
            
            # Find MuJoCo window
            def find_window(window, name):
                try:
                    wm_name = window.get_wm_name()
                    if wm_name and name in wm_name:
                        return window
                except:
                    pass
                try:
                    for child in window.query_tree().children:
                        result = find_window(child, name)
                        if result:
                            return result
                except:
                    pass
                return None
            
            mujoco_win = find_window(root, "MuJoCo")
            if mujoco_win:
                # Set fullscreen using _NET_WM_STATE
                wm_state = disp.intern_atom('_NET_WM_STATE')
                wm_fullscreen = disp.intern_atom('_NET_WM_STATE_FULLSCREEN')
                
                event = xdisplay.event.ClientMessage(
                    window=mujoco_win,
                    client_type=wm_state,
                    data=(32, [1, wm_fullscreen, 0, 0, 0])  # 1 = add
                )
                root.send_event(event, event_mask=X.SubstructureRedirectMask)
                disp.flush()
                print("Set window to fullscreen")
            else:
                print("Could not find MuJoCo window")
        except Exception as e:
            print(f"Could not set fullscreen: {e}")

        return self.viewer

    def __exit__(self, exc_type, exc_value, traceback):
        if self.viewer:
            self.viewer.close()
        if self.display_backup is not None:
            os.environ["DISPLAY"] = self.display_backup


if __name__ == "__main__":

    path = "assets/sample_scene.xml"
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)

    # set DISPLAY to :100
    display = ":100"
    print(f"Connecting to viewer at display {display}")
    with NetMujocoViewer(model, data, display=display) as viewer:
        while viewer.is_running():
            time.sleep(1)
