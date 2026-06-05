install_npm:
	cd net_mujoco/mujoco_wasm && npm install && npm run build

build_wheel: install_npm
	python -m pip wheel --no-deps --wheel-dir dist .
