# A simple tool to view your mujoco viewer through the web browser

This drastically simplifies development on clusters or remote dev servers.

# How does it work?
net_mujoco opens a virtual X display on your machine (by defaul :100). We then use xpra to mirror this display through html5 to a port on localhost.

**Important**: The Display is not password protected! Everyone with access to your machine could access the same screen at this instance!

# Installation: 

1. Make sure, xpra is installed on your system. If not available, install like so:

```bash
curl https://xpra.org/get-xpra.sh | bash
```

2. Install the project and its dependencies
```bash
pip install -e .
```


# Usage: 
1. On your remote machine, start the preconfigured Xpra server:
```bash
python -m net_mujoco.viewer
```

2. On another terminal, execute the net mujoco example:
```bash

python -m net_mujoco.net_mujoco
```

3. In your python code:
Replace the line where you would call:
```python
with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
    ...
```

with :
```python
from net_mujoco import NetMujocoViewer

with NetMujocoViewer(model, data, display) as viewer:
    time.sleep(10)  # Keep the viewer open for 10 seconds
```