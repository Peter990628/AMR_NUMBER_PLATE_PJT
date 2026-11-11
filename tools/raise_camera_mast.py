"""Raise the existing stereo sensor together and add a visual mast.

Run from Isaac Sim's Script Editor while stopped. Save under a new scene name
first to retain the low-camera comparison. The mast is visual geometry only;
it does not model added mass, stiffness, or collision volume.
"""

import math

from pxr import Gf, Usd, UsdGeom, UsdPhysics


def raise_camera_mast(stage, height_m=0.75):
    if not math.isfinite(height_m) or not 0.4 <= height_m <= 1.2:
        raise ValueError("Choose a camera height between 0.4 and 1.2 m.")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError("Expected a stage measured in meters.")
    if UsdGeom.GetStageUpAxis(stage) != "Z":
        raise RuntimeError("Expected a Z-up stage.")

    chassis_path = "/iw_hub_ROS/chassis"
    mount_path = chassis_path + "/front_hawk"
    mast_path = chassis_path + "/ExperimentCameraMast"
    mount = stage.GetPrimAtPath(mount_path)
    left = stage.GetPrimAtPath(mount_path + "/left/camera_left")
    right = stage.GetPrimAtPath(mount_path + "/right/camera_right")
    if not (mount and left and right):
        raise RuntimeError("Open the iw.hub camera test scene first.")
    if not (left.IsA(UsdGeom.Camera) and right.IsA(UsdGeom.Camera)):
        raise RuntimeError("The expected left and right Camera prims are missing.")
    if any(p.HasAPI(UsdPhysics.RigidBodyAPI) or p.IsA(UsdPhysics.Joint)
           for p in Usd.PrimRange(mount)):
        raise RuntimeError("This sensor assembly has physics bodies/joints; inspect its mount first.")
    existing = stage.GetPrimAtPath(mast_path)
    if existing and existing.GetCustomDataByKey("fixtureOwner") != "amr_camera_mast":
        raise RuntimeError("An unrelated prim already occupies " + mast_path)

    ops = UsdGeom.Xformable(mount).GetOrderedXformOps()
    if not ops or ops[0].GetOpName() != "xformOp:translate":
        raise RuntimeError("Expected front_hawk's existing leading translation operation.")
    translate = ops[0]
    cache = UsdGeom.XformCache()
    parent_world = cache.GetLocalToWorldTransform(mount.GetParent())
    vertical = parent_world.TransformDir(Gf.Vec3d(0, 0, 1))
    if not Gf.IsClose(vertical, Gf.Vec3d(0, 0, 1), 1e-5):
        raise RuntimeError("Use this visual mast with an upright, unscaled chassis.")
    current = cache.GetLocalToWorldTransform(left).ExtractTranslation()
    delta_world = Gf.Vec3d(0, 0, height_m - current[2])
    new_position = Gf.Vec3d(translate.Get()) + parent_world.GetInverse().TransformDir(delta_world)

    # Keep the support behind the optical centers. All dimensions are in the
    # chassis frame; the two-camera baseline and optical orientations stay intact.
    stereo_center_y = new_position[1] - 0.075
    post_x = new_position[0] - 0.06
    bottom_z = 0.12
    top_z = new_position[2] - 0.035
    if top_z <= bottom_z:
        raise RuntimeError("The requested camera height leaves no space for this mast.")

    with Usd.EditContext(stage, stage.GetRootLayer()):
        translate.Set(new_position)
        UsdGeom.Imageable(mount).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
        mast = UsdGeom.Xform.Define(stage, mast_path)
        mast.GetPrim().SetCustomDataByKey("fixtureOwner", "amr_camera_mast")
        mast.GetPrim().SetCustomDataByKey("purpose", "Visual support only; no added mass or colliders")

        def box(name, center, size):
            cube = UsdGeom.Cube.Define(stage, mast_path + "/" + name)
            cube.CreateSizeAttr(1.0)
            matrix = Gf.Matrix4d(1)
            matrix.SetScale(Gf.Vec3d(*size))
            matrix.SetTranslateOnly(Gf.Vec3d(*center))
            cube.MakeMatrixXform().Set(matrix)
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.10, 0.12)])

        box("Base", (post_x, stereo_center_y, bottom_z - 0.01), (0.12, 0.12, 0.02))
        box("Post", (post_x, stereo_center_y, (bottom_z + top_z) / 2),
            (0.035, 0.035, top_z - bottom_z))
        box("Top", (new_position[0] - 0.03, stereo_center_y, top_z), (0.12, 0.20, 0.012))

    cache = UsdGeom.XformCache()
    return {
        "left_camera_height_m": cache.GetLocalToWorldTransform(left).ExtractTranslation()[2],
        "right_camera_height_m": cache.GetLocalToWorldTransform(right).ExtractTranslation()[2],
        "raised_by_m": height_m - current[2],
    }


if __name__ == "<run_path>" or __name__ == "__main__":
    import omni.timeline
    import omni.usd

    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError("Press Stop before changing the sensor mount.")
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Open the camera test scene first.")
    result = raise_camera_mast(stage)
    for key, value in result.items():
        print(f"{key}: {value:.3f}")
    print("Stereo assembly raised. The support is visual geometry only.")
    print("Save, press Play, and compare the same /front_stereo_camera/left/image_raw topic.")
