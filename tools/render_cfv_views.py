#!/usr/bin/env python3
"""Render an isolated, stationary eight-view sedan fixture for CFV evaluation.

Run with Isaac Sim's python.sh, never from the active simulator's Script Editor.
Only the sedan source prim is referenced into a new in-memory stage. No AMR,
ROS bridge, source action graph, simulator control, or source-file save is used.
The geometric angle describes this fixture, not a claimed CFV dataset label.

Startup refuses to run alongside another visible Isaac/Kit process or with less
than 8 GiB MemAvailable. These checks have no override flag and run before Isaac
imports or output-directory creation. They are precautionary checks, not a hard
RAM/VRAM limit or a guarantee that rendering cannot exhaust system resources.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAR_PATH = "/World/sedan_car"
PLATE_PATH = CAR_PATH + "/ExperimentRearPlate/Face"
MIN_AVAILABLE_BYTES = 8 * 1024 ** 3


def resource_preflight(proc_root=Path("/proc")):
    """Check currently visible Linux processes and available RAM without Kit."""
    try:
        memory = dict(line.split(":", 1) for line in
                      (proc_root / "meminfo").read_text().splitlines())
        available = int(memory["MemAvailable"].split()[0]) * 1024
    except (OSError, KeyError, ValueError) as exc:
        raise RuntimeError("Cannot read MemAvailable; refusing Isaac startup.") from exc

    # Shell wrappers contain the full command as text; never match substrings in
    # their arguments. Exclude this process and every ancestor explicitly.
    excluded = set()
    pid = os.getpid()
    while pid > 0 and pid not in excluded:
        excluded.add(pid)
        try:
            status = (proc_root / str(pid) / "status").read_text()
            pid = int(next(line for line in status.splitlines()
                           if line.startswith("PPid:")).split()[1])
        except (OSError, ValueError, StopIteration):
            break
    running = []
    for process in proc_root.iterdir():
        if not process.name.isdigit() or int(process.name) in excluded:
            continue
        try:
            argv = (process / "cmdline").read_bytes().split(b"\0")
            executable = argv[0].decode(errors="replace")
        except (OSError, IndexError):
            continue  # Processes may exit while /proc is being inspected.
        name = Path(executable).name
        is_kit = name in ("kit", "kit.exe")
        if name.startswith("python"):
            # Standalone Isaac uses a Python executable. Detect its actual
            # loaded engine library, not the appearance of 'isaac' in source.
            try:
                with (process / "maps").open() as mappings:
                    is_kit = any("/kit/" in line and (
                        "/libcarb.so" in line or "libomni.kit.app" in line)
                        for line in mappings)
            except OSError:
                # A known Isaac Python executable with unavailable maps is
                # ambiguous; conservatively refuse a second startup.
                is_kit = "/kit/python/" in executable
        if is_kit:
            running.append(f"PID {process.name} ({executable})")
    reasons = []
    if running:
        reasons.append("another Isaac/Kit process is running: " + ", ".join(running))
    if available < MIN_AVAILABLE_BYTES:
        reasons.append(f"MemAvailable is {available / 1024 ** 3:.2f} GiB; at least 8 GiB is required")
    if reasons:
        raise RuntimeError("Refusing renderer startup: " + "; ".join(reasons)
                           + ". No simulator was started and no output directory was created.")
    return {"mem_available_bytes": available,
            "minimum_available_bytes": MIN_AVAILABLE_BYTES,
            "process_check": "No other Isaac/Kit process found in the visible /proc namespace.",
            "limit": "Preflight is not a RAM/VRAM reservation or a runtime resource guarantee."}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "amr_camera_fail.usd")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--camera-info-stream", type=Path, default=ROOT / (
        "local_artifacts/recovery_observation_2026-11-13/failure_01/streams.jsonl"))
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--camera-height-m", type=float, default=0.75)
    parser.add_argument("--radius-m", type=float, default=5.5)
    parser.add_argument("--frames-per-view", type=int, default=3)
    parser.add_argument("--warmup-frames", type=int, default=12)
    parser.add_argument("--angles", type=float, nargs="+", default=list(range(0, 360, 45)))
    args = parser.parse_args()
    if min(args.width, args.height, args.frames_per_view, args.warmup_frames) <= 0:
        parser.error("Resolution, frame count and warmup count must be positive.")
    if not math.isfinite(args.radius_m) or args.radius_m < 3:
        parser.error("Use a finite radius of at least 3 m.")
    if not math.isfinite(args.camera_height_m) or args.camera_height_m <= 0:
        parser.error("Camera height must be positive and finite.")
    if not all(math.isfinite(a) for a in args.angles):
        parser.error("Angles must be finite.")
    if len(set(a % 360 for a in args.angles)) != len(args.angles):
        parser.error("Angles must be unique modulo 360.")
    args.source = args.source.resolve(strict=True)
    args.output_dir = args.output_dir.resolve()
    if args.output_dir.exists():
        parser.error("Output directory already exists; choose a new directory.")
    return args


def calibration(args):
    """Match recorded pinhole K and aspect ratio; do not imply lens equivalence."""
    message = None
    with args.camera_info_stream.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("stream") == "camera_info" and record.get("message", {}).get("k"):
                message = record["message"]
                break
    if message is None:
        raise RuntimeError("No recorded CameraInfo found; cannot match camera calibration.")
    sx, sy = args.width / message["width"], args.height / message["height"]
    k = message["k"]
    if not math.isclose(sx, sy, rel_tol=1e-6):
        raise RuntimeError("Output must preserve recorded camera aspect ratio.")
    if not (math.isclose(k[2], message["width"] / 2, abs_tol=1e-3)
            and math.isclose(k[5], message["height"] / 2, abs_tol=1e-3)):
        raise RuntimeError("This capture tool expects a centered source principal point.")
    return {
        "source": str(args.camera_info_stream.resolve()),
        "source_message": message,
        "resolution": [args.width, args.height],
        "fx": k[0] * sx, "fy": k[4] * sy,
        "cx": k[2] * sx, "cy": k[5] * sy,
        "model": "centered pinhole; recorded distortion coefficients are not applied",
    }


def main():
    args = arguments()
    try:
        preflight = resource_preflight()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    intrinsics = calibration(args)
    source_hash = hashlib.sha256(args.source.read_bytes()).hexdigest()
    args.output_dir.mkdir(parents=True, exist_ok=False)

    # Import simulator-dependent modules only after creating the separate app.
    from isaacsim import SimulationApp
    app = SimulationApp({
        "headless": True, "renderer": "RaytracedLighting", "multi_gpu": False,
        "width": args.width, "height": args.height,
        "window_width": args.width, "window_height": args.height,
        "hide_ui": True,
    })
    try:
        import carb.settings
        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from PIL import Image
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

        # Opening this masked USD is a CPU read. It is never attached to Kit.
        source = Usd.Stage.OpenMasked(
            str(args.source), Usd.StagePopulationMask([CAR_PATH]))
        if UsdGeom.GetStageUpAxis(source) != "Z" or not math.isclose(
                UsdGeom.GetStageMetersPerUnit(source), 1.0):
            raise RuntimeError("Source must be meter-scaled and Z-up.")
        source_car = source.GetPrimAtPath(CAR_PATH)
        if not source_car:
            raise RuntimeError("Source sedan prim is absent.")
        parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(source_car.GetParent())
        if not Gf.IsClose(parent_world, Gf.Matrix4d(1), 1e-8):
            raise RuntimeError("Source sedan parent must have identity world transform.")
        source_layers = [l.realPath for l in source.GetUsedLayers() if not l.anonymous]
        if any(not Path(p).is_file() for p in source_layers):
            raise RuntimeError("All source layers must be local files.")

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        car = stage.DefinePrim(CAR_PATH, "Xform")
        car.GetReferences().AddReference(str(args.source), CAR_PATH)
        if any("Graph" in p.GetTypeName() or "ROS" in str(p.GetPath())
               for p in stage.Traverse()):
            raise RuntimeError("Unexpected action graph or robot prim in isolated fixture.")

        cache = UsdGeom.XformCache()
        plate = stage.GetPrimAtPath(PLATE_PATH)
        if not plate:
            raise RuntimeError("Rear plate missing; cannot establish vehicle front direction.")
        plate_world = cache.GetLocalToWorldTransform(plate)
        rear_normal = plate_world.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        forward = Gf.Vec3d(-rear_normal[0], -rear_normal[1], 0).GetNormalized()
        left = Gf.Cross(Gf.Vec3d(0, 0, 1), forward).GetNormalized()
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        bbox = bounds.ComputeWorldBound(car).ComputeAlignedRange()
        center = bbox.GetMidpoint()
        plate_position = plate_world.ExtractTranslation()
        if Gf.Dot(plate_position - center, forward) >= 0:
            raise RuntimeError("Rear plate normal/position disagree about the vehicle front.")

        # A neutral floor and lights replace the original stage environment.
        floor = UsdGeom.Mesh.Define(stage, "/World/TestFloor")
        x, y = float(center[0]), float(center[1])
        floor.CreatePointsAttr([(x-30, y-30, 0), (x+30, y-30, 0),
                                (x+30, y+30, 0), (x-30, y+30, 0)])
        floor.CreateFaceVertexCountsAttr([4])
        floor.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        floor.CreateNormalsAttr([(0, 0, 1)] * 4)
        floor.SetNormalsInterpolation("vertex")
        material = UsdShade.Material.Define(stage, "/World/FloorMaterial")
        shader = UsdShade.Shader.Define(stage, "/World/FloorMaterial/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.65, 0.65, 0.65))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.75)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material)
        dome = UsdLux.DomeLight.Define(stage, "/World/FillLight")
        dome.CreateIntensityAttr(350)
        sun = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
        sun.CreateIntensityAttr(2500)
        sun.CreateAngleAttr(3)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set((-35, -25, 0))

        camera = UsdGeom.Camera.Define(stage, "/World/TestCamera")
        focal_length = 24.0
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        camera.CreateFocalLengthAttr(focal_length)
        camera.CreateHorizontalApertureAttr(focal_length * args.width / intrinsics["fx"])
        camera.CreateVerticalApertureAttr(focal_length * args.height / intrinsics["fy"])
        camera.CreateClippingRangeAttr((0.05, 1000))
        camera_transform = camera.MakeMatrixXform()
        settings = carb.settings.get_settings()
        settings.set("/omni/replicator/captureOnPlay", False)
        settings.set("/rtx/post/dlss/execMode", 2)
        product = rep.create.render_product(str(camera.GetPath()), (args.width, args.height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)

        manifest = {
            "schema_version": 1,
            "resource_preflight": preflight,
            "purpose": "isolated synthetic eight-view orientation check; no robot motion",
            "source_scene": str(args.source), "source_scene_sha256": source_hash,
            "source_layers": source_layers,
            "source_prim": CAR_PATH,
            "vehicle_bbox_min_world_m": list(bbox.GetMin()),
            "vehicle_bbox_max_world_m": list(bbox.GetMax()),
            "vehicle_center_world_m": list(center),
            "vehicle_forward_world": list(forward), "vehicle_left_world": list(left),
            "rear_plate_world_m": list(plate_position),
            "rear_plate_outward_normal_world": list(rear_normal),
            "heading_evidence": "Vehicle forward is opposite horizontal rear-plate face normal; rear plate is behind bbox center.",
            "angle_definition": "Camera azimuth around vehicle bbox center: front=0, vehicle right=90, rear=180, vehicle left=270 deg.",
            "reference_limit": "geometric_reference_deg is exact for this synthetic camera placement; it is not an authoritative CFV annotation.",
            "camera_intrinsics": intrinsics,
            "camera_height_m": args.camera_height_m, "camera_radius_m": args.radius_m,
            "frames_per_view": args.frames_per_view,
            "limitations": [
                "One static sedan only; repeated frames are not independent viewpoints.",
                "Neutral test lights/floor differ from the saved ROS observation scene.",
                "Pinhole K is scaled from recorded CameraInfo, but lens distortion is omitted.",
                "Camera points horizontally toward bbox center at camera height.",
                "No source scene was opened in the active simulator or saved by this tool.",
            ],
            "samples": [],
        }
        target = Gf.Vec3d(center[0], center[1], args.camera_height_m)
        for requested_angle in args.angles:
            angle = requested_angle % 360
            radians = math.radians(angle)
            position = target + args.radius_m * (
                math.cos(radians) * forward - math.sin(radians) * left)
            view = Gf.Matrix4d().SetLookAt(position, target, Gf.Vec3d(0, 0, 1))
            camera_world = view.GetInverse()
            camera_transform.Set(camera_world)
            for _ in range(args.warmup_frames):
                app.update()
            for index in range(args.frames_per_view):
                rep.orchestrator.step(rt_subframes=4, delta_time=0.0)
                data = np.asarray(rgb.get_data())
                if data.shape[:2] != (args.height, args.width) or data.size == 0:
                    raise RuntimeError(f"Unexpected RGB shape at {angle}: {data.shape}")
                if not np.isfinite(data).all() or float(data[..., :3].std()) < 1:
                    raise RuntimeError(f"Blank/nonfinite RGB at {angle}")
                group = f"view_{angle:07.3f}".replace(".", "p")
                filename = f"{group}_{index:02d}.png"
                image_path = args.output_dir / filename
                Image.fromarray(data).convert("RGB").save(image_path)
                sample = {
                    "id": f"{group}_{index:02d}", "group": group,
                    "image": str(image_path), "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                    "geometric_reference_deg": angle,
                    "camera_world_xyz_m": list(position),
                    "camera_look_at_world_xyz_m": list(target),
                    "camera_world_matrix_rows": [list(row) for row in camera_world],
                    "vehicle_forward_camera_xyz": list(view.TransformDir(forward)),
                    "camera_axes": "USD camera: +X image right, +Y image up, -Z optical forward",
                    "frame_index": index,
                }
                manifest["samples"].append(sample)
                print(f"CFV_CAPTURE {filename} geometry={angle:.1f}", flush=True)
            (args.output_dir / "manifest.json").write_text(
                json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        rep.orchestrator.wait_until_complete()
        rgb.detach(product)
        if hashlib.sha256(args.source.read_bytes()).hexdigest() != source_hash:
            raise RuntimeError("Source hash changed during capture; investigate external edits.")
        manifest["complete"] = True
        (args.output_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"CFV_CAPTURE_COMPLETE {len(manifest['samples'])} images at {args.output_dir}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
