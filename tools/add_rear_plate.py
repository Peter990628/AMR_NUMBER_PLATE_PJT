"""Attach the synthetic rear plate to the imported sedan in Isaac Sim's GUI.

Run with runpy.run_path from Window > Script Editor while stopped.
The current scene is edited in memory; save it using Ctrl+S after inspection.
The placement is fitted to this sedan's existing rear number-plate mesh.
"""

import json
import math
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


def attach_plate(stage):
    asset_dir = Path(__file__).resolve().parents[1] / "assets" / "license_plate"
    asset = asset_dir / "plate_123ga4567.usda"
    texture = asset_dir / "plate_123ga4567.png"
    if not asset.is_file() or not texture.is_file():
        raise RuntimeError("Plate USD or texture is missing.")
    mount = json.loads((asset_dir / "mount.json").read_text(encoding="utf-8"))
    car_path = "/World/sedan_car"
    plate_path = car_path + "/ExperimentRearPlate"
    car = stage.GetPrimAtPath(car_path)
    if not car:
        raise RuntimeError("Open amr_vehicle_test.usd with /World/sedan_car first.")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError("This fixture expects a stage measured in meters.")
    parent_world = UsdGeom.XformCache().GetLocalToWorldTransform(car)
    axis_lengths = [
        parent_world.TransformDir(Gf.Vec3d(*axis)).GetLength()
        for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    ]
    if not all(math.isclose(length, 0.01, rel_tol=1e-5) for length in axis_lengths):
        raise RuntimeError("The sedan scale changed; recheck its centimeter-to-meter conversion.")
    existing = stage.GetPrimAtPath(plate_path)
    if existing and existing.GetCustomDataByKey("fixtureOwner") != "amr_plate_experiment":
        raise RuntimeError("An unrelated prim already occupies " + plate_path)

    # USD row-vector convention. The referenced plate is in meters, while the
    # car's child coordinates are centimeters. These rows also orient the text
    # right-side-up when the rear plate is viewed from outside the vehicle.
    rows = [[100.0 * x for x in mount[key]] + [0.0]
            for key in ("right", "up", "normal")]
    rows.append(mount["center_cm"] + [1.0])
    local = Gf.Matrix4d(*(value for row in rows for value in row))
    root_layer = stage.GetRootLayer()
    reference = str(asset)
    if not root_layer.anonymous and root_layer.realPath:
        reference = os.path.relpath(asset, Path(root_layer.realPath).parent)

    with Usd.EditContext(stage, root_layer):
        plate = UsdGeom.Xform.Define(stage, plate_path)
        plate.GetPrim().GetReferences().SetReferences([Sdf.Reference(reference)])
        plate.MakeMatrixXform().Set(local)
        plate.GetPrim().SetCustomDataByKey("fixtureOwner", "amr_plate_experiment")
        plate.GetPrim().SetCustomDataByKey("label", mount["label"])
    return plate_path


if __name__ == "<run_path>" or __name__ == "__main__":
    import omni.timeline
    import omni.usd

    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError("Press Stop before adding the plate.")
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Open the vehicle scene first.")
    result = attach_plate(stage)
    context.get_selection().set_selected_prim_paths([result], True)
    print("Attached 123가4567 (0.52 x 0.11 m): " + result)
    print("Inspect the rear plate, then press Ctrl+S to save the scene.")
