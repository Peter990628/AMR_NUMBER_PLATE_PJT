"""Run once per Isaac Sim session in Script Editor, before or during Play."""
import asyncio
import builtins
from datetime import datetime, timezone
import json
from pathlib import Path
import runpy

import omni.timeline
import omni.usd


HERE = Path(__file__).resolve().parent
bridge = runpy.run_path(str(HERE / 'recovery_lidar_capture.py'))
LOCK = '_peter_recovery_lidar_watcher'


async def watch():
    timeline = omni.timeline.get_timeline_interface()
    context = omni.usd.get_context()
    while True:
        await asyncio.sleep(.25)
        for ticket in bridge['REQUESTS'].glob('*.json'):
            if not bridge['active_request'](ticket):
                ticket.unlink(missing_ok=True)
                continue
            stage = context.get_stage()
            if stage is None or not timeline.is_playing():
                continue
            try:
                manifest = ticket.resolve(strict=True)
                sensor = bridge['SENSORS'][json.loads(manifest.read_text())['robot']]
            except FileNotFoundError:
                continue  # The run ended before it could be claimed.
            directory = manifest.parent
            if not stage.GetPrimAtPath(sensor):
                continue
            try:
                (directory / 'claimed').touch(exist_ok=False)
            except FileExistsError:
                continue
            try:
                module = runpy.run_path(str(HERE / 'inspect_lidar_pipeline_in_gui.py'),
                                        run_name='recovery_lidar_reader',
                                        init_globals={'RECOVERY_CAPTURE_DIR': directory, 'RECOVERY_SENSOR': sensor})
                existing = getattr(builtins, module['LOCK'], None)
                if existing is not None and not existing.done():
                    raise RuntimeError('Another internal LiDAR reader is active.')
                task = asyncio.ensure_future(module['inspect_pipeline'](
                    run_active=lambda: bridge['active_request'](manifest)))
                setattr(builtins, module['LOCK'], task)
                await task
            except Exception as exc:
                report = {'status': 'error', 'error': str(exc),
                          'finished_utc': datetime.now(timezone.utc).isoformat()}
                tmp = directory / 'report.tmp'
                tmp.write_text(json.dumps(report))
                tmp.replace(directory / 'report.json')


existing = getattr(builtins, LOCK, None)
if existing is None or existing.done():
    setattr(builtins, LOCK, asyncio.ensure_future(watch()))
print('Recovery LiDAR reader listening. Start the normal recovery command during Play.')
