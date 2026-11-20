"""Record existing LiDAR GPU buffers around the first missing-sector event.

Keep Play active. Waits at most 120 seconds, retains up to 3 seconds before
the event and 5 seconds after it, and never moves the robot or edits the scene.
"""
import runpy
from pathlib import Path

runpy.run_path(str(Path(__file__).with_name('inspect_lidar_pipeline_in_gui.py')),
              init_globals={'CAPTURE_GPU_PAYLOADS': True, 'CAPTURE_DROPOUT_WINDOW': True})
