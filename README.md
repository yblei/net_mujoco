# Net MuJoCo

A simple tool to view your MuJoCo simulations through a web browser.

This drastically simplifies development on clusters or remote dev servers by providing browser-based visualization without requiring local X11 forwarding.

## How Does It Work?

Net MuJoCo opens a virtual X display on your machine (by default `:100`). It then uses Xpra to mirror this display through HTML5 to a port on localhost, making it accessible via your web browser.

**⚠️ Security Note**: The display is not password protected! Anyone with access to your machine can view the same screen.

## Installation

### 1. Install Xpra

Make sure Xpra is installed on your system:

```bash
# Using Xpra installer script
curl https://xpra.org/get-xpra.sh | bash

# Or via package manager (Ubuntu/Debian)
sudo apt-get install xpra
```

### 2. Install Net MuJoCo

```bash
pip install -e .
```

## Usage

### Quick Start

1. **Start the Xpra server** (in one terminal):

   ```bash
   python -m net_mujoco.server
   ```

2. **Run your MuJoCo simulation** (in another terminal):

   ```bash
   python -m net_mujoco.net_mujoco
   ```

3. **Open your browser** - The viewer will automatically open at `http://localhost:8080`

### In Your Code

Replace the standard MuJoCo viewer:

```python
# Before
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

With Net MuJoCo:

```python
# After
from net_mujoco import NetMujocoViewer

with NetMujocoViewer(model, data, display=":100") as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### CLI Options

The server supports various configuration options:

```bash
# Custom port
python -m net_mujoco.server --port 9000

# Verbose output
python -m net_mujoco.server --verbose

# Show all options
python -m net_mujoco.server --help
```

## Features

- 🌐 Browser-based MuJoCo visualization
- 🖥️ Full-screen support
- 🔧 Easy-to-use context manager API
- 🚀 Configurable quality and performance settings
- 📡 Works seamlessly with VS Code Remote SSH

## License

MIT
