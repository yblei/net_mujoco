#!/usr/bin/env python3
"""
Example: MuJoCo Web Viewer with launch_passive interface

This demonstrates how to use the web viewer with an interface similar to
mujoco.viewer.launch_passive. Physics runs in Python, visualization in browser.
"""

import time
import mujoco
from net_mujoco import launch_passive
import numpy as np


def main():
    # Load model
    # Note: Complex models like fr3 may be too large for WASM memory
    # Use simpler models for web visualization
    import os
    
    # Choose which model to load:
    # Option 1: Simple sphere (works well)
    # full_path = os.path.abspath('assets/sample_scene.xml')
    
    # Option 2: FR3 robot arm (more complex, needs mesh files)
    # IMPORTANT: Use absolute path so mesh files can be found
    xml_path = 'assets/sample_scene.xml'
    full_path = os.path.abspath(xml_path)
    
    #m = mujoco.MjModel.from_xml_path(full_path)
    mjspec = mujoco.MjSpec.from_file(full_path)
    m = mjspec.compile()
    d = mujoco.MjData(m)
    
    print("=" * 60)
    print("MuJoCo Web Viewer - Passive Mode Example")
    print("=" * 60)
    print("\nThen open browser at: http://yblei.github.io/net_mujoco/")
    print(f"\nLoading model from: {full_path}")
    print("Starting simulation...")
    print("Press Ctrl+C to stop.\n")
    
    # Launch passive viewer (similar to mujoco.viewer.launch_passive)
    # Pass full_path so we can find mesh files in the same directory
    with launch_passive(m, d, model_source=mjspec, open_browser=True) as viewer:
        start_time = time.time()
        step_count = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # Set joint position directly (no actuators in simple scene)
            if m.nq > 0:
                d.qpos[0] = 3 * np.sin(1 * d.time)
            
            # Step the physics
            mujoco.mj_step(m, d)
            step_count += 1
            
            # Sync with viewer to update visualization
            viewer.sync()
            
            # Print status every 60 steps (roughly once per second at 60Hz)
            if step_count % 60 == 0:
                elapsed = time.time() - start_time
                sim_fps = step_count / elapsed if elapsed > 0 else 0
                print(f"Sim time: {d.time:.2f}s | Steps: {step_count} | Sim FPS: {sim_fps:.1f}")
            
            # Maintain real-time rate
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user.")
