import subprocess
import sys
import time
import typer

app = typer.Typer()

def launch_server(ip: str, port: int, verbose: bool = False):
    """Launch an Xpra server on the specified IP and port (non-blocking)."""
    process = subprocess.Popen(
        ["xpra", "start", ":100", f"--bind-tcp={ip}:{port}",
         "--html=on", "--opengl=yes", "--encodings=jpeg,png", "--daemon=no", "--dpi=150"],
        stdout=sys.stdout if verbose else subprocess.PIPE,
        stderr=sys.stderr if verbose else subprocess.PIPE,
    )
    return process


@app.command()
def main(
    ip: str = typer.Option("localhost", help="IP address to bind to"),
    port: int = typer.Option(8080, help="Port number to bind to"),
    display: str = typer.Option(":100", help="X display number"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Show detailed Xpra output"),
):
    """🚀 Launch an Xpra HTML5 server for MuJoCo visualization."""
    
    if verbose:
        typer.echo("Verbose mode enabled\n")
    
    typer.echo("=" * 50)
    typer.echo("Starting Xpra Server")
    typer.echo("=" * 50)
    typer.echo(f"IP:      {ip}")
    typer.echo(f"Port:    {port}")
    typer.echo(f"Display: {display}")
    typer.echo("=" * 50)
    typer.echo("")
    
    typer.echo("Starting server...")
    server_process = launch_server(ip, port, verbose=verbose)
    time.sleep(1)
    
    if server_process.poll() is None:
        typer.echo("✓ Server started!")
        typer.echo("")
        typer.echo(f"✓ PID:    {server_process.pid}")
        typer.echo(f"✓ URL:    http://{ip}:{port}")
        typer.echo(f"⚡ Status: Running")
        typer.echo("")
        typer.echo("Press Ctrl+C to stop the server")
        typer.echo("")
        time.sleep(5)
        import webbrowser
        webbrowser.open(f"http://{ip}:{port}")
    else:
        typer.echo("❌ Server failed to start", err=True)
        if not verbose:
            stdout, stderr = server_process.communicate()
            if stderr:
                typer.echo(f"Error: {stderr}", err=True)
        raise typer.Exit(code=1)
    
    try:
        while True:
            time.sleep(1)
            if server_process.poll() is not None:
                typer.echo(f"❌ Server exited with code {server_process.returncode}", err=True)
                if not verbose:
                    stdout, stderr = server_process.communicate()
                    if stderr:
                        typer.echo(f"Error output:\n{stderr}", err=True)
                raise typer.Exit(code=server_process.returncode)
    except KeyboardInterrupt:
        typer.echo("\n\nStopping server...")
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
            typer.echo("✅ Server stopped successfully")
        except subprocess.TimeoutExpired:
            typer.echo("⚠️  Killing server...")
            server_process.kill()
            server_process.wait()
            typer.echo("✅ Server killed")


if __name__ == "__main__":
    app()