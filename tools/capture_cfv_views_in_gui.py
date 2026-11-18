"""Temporarily pose the sedan for four CFV images in the existing Isaac GUI.

Run in Script Editor with runpy.run_path while Play is active. This reuses the
existing camera/ROS publisher; an external subscriber captures and acknowledges
each view. No simulator, render product, camera, or motion publisher is created.
Only an owned anonymous session sublayer is removed on completion or failure.
Do not save or edit the scene while this short capture job is running.
"""

import asyncio
import builtins
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ROOT = ROOT / "local_artifacts/cfv_test_2026-11-13"
CAR_PATH = "/World/sedan_car"
CAMERA_PATH = "/iw_hub_ROS/chassis/front_hawk/left/camera_left"
PLATE_PATH = CAR_PATH + "/ExperimentRearPlate/Face"
ANGLES = (225, 180, 135, 0)
VIEW_NAMES = {225: "Rear-left diagonal", 180: "Rear", 135: "Rear-right diagonal", 0: "Front"}
LOCK = "_cfv_gui_capture_task"


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def vector(value):
    return [float(x) for x in value]


async def capture_views():
    import omni.kit.app
    import omni.timeline
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    job_dir = OUTPUT_ROOT / ("gui_views_" + stamp + "_" + uuid.uuid4().hex[:6])
    job_dir.mkdir(parents=True, exist_ok=False)
    state_path = job_dir / "state.json"
    state = {"phase": "starting", "job_dir": str(job_dir), "restored": False,
             "created_at_utc": datetime.now(timezone.utc).isoformat()}
    write_json(state_path, state)
    write_json(OUTPUT_ROOT / "gui_test_latest.json", {"job_dir": str(job_dir)})
    app = omni.kit.app.get_app()
    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    stage = context.get_stage()
    session, temporary, initial_car = None, None, None
    records = []

    def ensure_play():
        if context.get_stage() != stage or not timeline.is_playing():
            raise RuntimeError("Keep the same stage open and Play active during capture.")

    def geometry():
        cache = UsdGeom.XformCache()
        car_matrix = cache.GetLocalToWorldTransform(car)
        camera_matrix = cache.GetLocalToWorldTransform(camera)
        plate_matrix = cache.GetLocalToWorldTransform(plate)
        normal = plate_matrix.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        forward = Gf.Vec3d(-normal[0], -normal[1], 0).GetNormalized()
        return car_matrix, camera_matrix, forward

    def camera_unchanged(current):
        if (current.ExtractTranslation() - original_camera.ExtractTranslation()).GetLength() > 0.01:
            raise RuntimeError("Camera moved more than 1 cm; this capture requires a stationary AMR.")
        for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)):
            a = current.TransformDir(axis).GetNormalized()
            b = original_camera.TransformDir(axis).GetNormalized()
            if math.acos(max(-1.0, min(1.0, Gf.Dot(a, b)))) > 0.01:
                raise RuntimeError("Camera rotated more than 0.01 rad; capture aborted.")

    try:
        if stage is None:
            raise RuntimeError("Open amr_camera_fail.usd first.")
        ensure_play()
        if UsdGeom.GetStageUpAxis(stage) != "Z" or not math.isclose(
                UsdGeom.GetStageMetersPerUnit(stage), 1.0):
            raise RuntimeError("The capture fixture requires a Z-up, meter-scaled stage.")
        car, camera, plate = [stage.GetPrimAtPath(path) for path in
                              (CAR_PATH, CAMERA_PATH, PLATE_PATH)]
        if not all((car, camera, plate)) or not camera.IsA(UsdGeom.Camera):
            raise RuntimeError("Required sedan, rear plate, or existing camera is missing.")
        cache = UsdGeom.XformCache()
        parent_matrix = cache.GetLocalToWorldTransform(car.GetParent())
        if not Gf.IsClose(parent_matrix, Gf.Matrix4d(1.0), 1e-8):
            raise RuntimeError("Sedan parent must have an identity world transform.")
        for prim in Usd.PrimRange(car):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                if UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is not False:
                    raise RuntimeError("Sedan has an active rigid body; only a static fixture is supported.")
        xform = UsdGeom.Xformable(car)
        ops = xform.GetOrderedXformOps()
        op_names = [str(op.GetOpName()) for op in ops]
        expected_prefix = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
        conversion_suffix = ["xformOp:rotateX:unitsResolve", "xformOp:scale:unitsResolve"]
        if xform.GetResetXformStack() or op_names not in (
                expected_prefix, expected_prefix + conversion_suffix):
            raise RuntimeError("Expected translate, rotateXYZ, scale before asset conversion ops.")
        translation, rotation = ops[0].GetAttr(), ops[1].GetAttr()
        initial_translation, initial_rotation = translation.Get(), rotation.Get()
        if any(abs(float(v)) > 1e-6 for v in initial_rotation[:2]):
            raise RuntimeError("Sedan must be upright (rotateXYZ x/y must be zero).")
        if any(op.GetAttr().GetNumTimeSamples() for op in ops):
            raise RuntimeError("Animated sedan transforms are unsupported.")
        session = stage.GetSessionLayer()
        for attr in (translation, rotation):
            if any(spec.layer == session and spec.HasDefaultValue()
                   for spec in attr.GetPropertyStack()):
                raise RuntimeError("Sedan transform is authored directly in the session layer; stop capture.")

        initial_car, original_camera, initial_forward = geometry()
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        center = bbox.ComputeWorldBound(car).ComputeAlignedRange().GetMidpoint()
        center_local = initial_car.GetInverse().Transform(center)
        rear_position = cache.GetLocalToWorldTransform(plate).ExtractTranslation()
        if Gf.Dot(rear_position - center, initial_forward) >= 0:
            raise RuntimeError("Rear plate position and outward normal do not agree.")
        camera_position = original_camera.ExtractTranslation()
        optical_direction = original_camera.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
        heading = Gf.Vec3d(optical_direction[0], optical_direction[1], 0)
        if heading.GetLength() < 0.1:
            raise RuntimeError("Camera must look approximately horizontally.")
        heading.Normalize()
        target_center = camera_position + heading * 5.5
        target_center[2] = center[2]
        initial_heading = math.atan2(initial_forward[1], initial_forward[0])
        toward_camera_heading = math.atan2(-heading[1], -heading[0])
        source = stage.GetRootLayer().realPath
        metadata = {"source_stage": source,
                    "source_sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest() if source else None,
                    "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    "car_path": CAR_PATH, "camera_path": CAMERA_PATH,
                    "requested_angles_deg": list(ANGLES), "horizontal_radius_m": 5.5,
                    "initial_center_world": vector(center), "target_center_world": vector(target_center),
                    "reference_scope": "Controlled vehicle-center geometry, not CFV human labels or plate incidence.",
                    "camera_vertical_pitch_deg": math.degrees(math.asin(optical_direction[2]))}
        write_json(job_dir / "setup.json", metadata)
        state.update(source_stage=source, source_sha256=metadata["source_sha256"],
                     setup_file=str(job_dir / "setup.json"))
        temporary = Sdf.Layer.CreateAnonymous("cfv_gui_capture.usda")
        session.subLayerPaths.insert(0, temporary.identifier)

        for index, angle in enumerate(ANGLES):
            ensure_play()
            camera_unchanged(geometry()[1])
            view_id = "view_" + str(angle)
            state.update(phase="positioning", view_id=view_id, view_index=index)
            write_json(state_path, state)
            delta_deg = math.degrees(toward_camera_heading + math.radians(angle) - initial_heading)
            expected_rotation = type(initial_rotation)(0, 0, initial_rotation[2] + delta_deg)
            with Usd.EditContext(stage, temporary):
                rotation.Set(expected_rotation)
                translation.Set(initial_translation)
                moved_center = geometry()[0].Transform(center_local)
                target_translation = Gf.Vec3d(initial_translation) + target_center - moved_center
                translation.Set(type(initial_translation)(*target_translation))
            if not Gf.IsClose(rotation.Get(), expected_rotation, 1e-5):
                raise RuntimeError("Temporary rotation did not compose; stronger transform opinion exists.")
            started, frames = time.monotonic(), 0
            while frames < 60 or time.monotonic() - started < 2.0:
                ensure_play()
                if time.monotonic() - started > 60.0:
                    raise TimeoutError("Camera warm-up did not finish within 60 seconds.")
                await app.next_update_async()
                frames += 1
            actual_car, actual_camera, forward = geometry()
            camera_unchanged(actual_camera)
            actual_center = actual_car.Transform(center_local)
            if (actual_center - target_center).GetLength() > 0.001:
                raise RuntimeError("Sedan center differs from requested fixture by more than 1 mm.")
            toward_camera = actual_camera.ExtractTranslation() - actual_center
            toward_camera[2] = 0
            toward_camera.Normalize()
            left = Gf.Cross(Gf.Vec3d(0, 0, 1), forward)
            reference = math.degrees(math.atan2(-Gf.Dot(toward_camera, left),
                                                Gf.Dot(toward_camera, forward))) % 360
            if abs((reference - angle + 180) % 360 - 180) > 0.01:
                raise RuntimeError("Actual geometric reference does not match requested angle.")
            record = {"view_id": view_id, "display_name": VIEW_NAMES[angle],
                      "view_index": index, "requested_angle_deg": angle,
                      "geometric_reference_deg": reference, "vehicle_forward_world": vector(forward),
                      "vehicle_center_world": vector(actual_center),
                      "camera_matrix_world": [vector(row) for row in actual_camera],
                      "ready_at_utc": datetime.now(timezone.utc).isoformat(),
                      "simulation_time_seconds": timeline.get_current_time(),
                      "warmup_updates": frames, "warmup_wall_seconds": time.monotonic() - started}
            state.update(record, phase="ready")
            write_json(state_path, state)
            deadline = time.monotonic() + 120.0
            while True:
                ensure_play()
                current_car, current_camera, _ = geometry()
                camera_unchanged(current_camera)
                if not Gf.IsClose(current_car, actual_car, 1e-5):
                    raise RuntimeError("Sedan was edited while waiting for image capture.")
                ack_path = job_dir / "ack.json"
                try:
                    ack = json.loads(ack_path.read_text()) if ack_path.exists() else {}
                except (OSError, json.JSONDecodeError):
                    ack = {}
                if ack.get("status") == "abort":
                    raise RuntimeError("External capture requested abort.")
                if ack.get("view_id") == view_id and ack.get("status") == "captured":
                    record["capture_ack"] = ack
                    records.append(record)
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("No matching capture acknowledgement within 120 seconds: " + view_id)
                await app.next_update_async()
            write_json(job_dir / "views.json", {"setup": metadata, "views": records})
        state.update(phase="complete", captured_view_count=len(records))
    except BaseException as exc:
        state.update(phase="error", error=type(exc).__name__ + ": " + str(exc))
        print("CFV capture stopped: " + state["error"])
    finally:
        if session is not None and temporary is not None:
            session.subLayerPaths = [path for path in session.subLayerPaths if path != temporary.identifier]
        restored = initial_car is None or Gf.IsClose(
            UsdGeom.XformCache().GetLocalToWorldTransform(car), initial_car, 1e-7)
        if not restored:
            state.update(phase="error", restoration_error="Original sedan transform did not restore.")
        state.update(restored=restored, finished_at_utc=datetime.now(timezone.utc).isoformat())
        write_json(state_path, state)
        print("CFV capture " + state["phase"] + "; restored=" + str(restored) + ". " + str(job_dir))


if __name__ in ("__main__", "<run_path>"):
    existing = getattr(builtins, LOCK, None)
    if existing is not None and not existing.done():
        raise RuntimeError("A CFV GUI capture job is already running.")
    setattr(builtins, LOCK, asyncio.ensure_future(capture_views()))
    print("CFV four-view capture started in this GUI; keep Play active and do not save/edit the scene.")
    print("External subscriber must capture each ready view and acknowledge it within 120 seconds.")
