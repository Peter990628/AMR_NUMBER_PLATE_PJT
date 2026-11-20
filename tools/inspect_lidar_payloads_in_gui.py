"""Run the bounded existing-graph reader with GPU payload copies enabled."""
import runpy
from pathlib import Path

runpy.run_path(str(Path(__file__).with_name('inspect_lidar_pipeline_in_gui.py')),
              init_globals={'CAPTURE_GPU_PAYLOADS': True})
