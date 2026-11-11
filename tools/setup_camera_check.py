"""Arrange a stationary plate target in front of the existing left RGB camera.

Run in Isaac Sim's Script Editor after saving the scene under a test filename.
This edits only the parked car's position and yaw. Sensor configuration, robot
pose, and ROS graphs stay unchanged. Target geometry is used for fixture setup,
not supplied to a robot controller or perception algorithm.
"""

import math

from pxr import Gf, Usd, UsdGeom


def setup_camera_check(stage, distance_m=1.5):
    if not math.isfinite(distance_m) or distance_m < 1.0:
        raise ValueError("Use a horizontal camera-to-plate distance of at least 1 m.")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError("This setup expects a stage measured in meters.")
    if UsdGeom.GetStageUpAxis(stage) != "Z":
        raise RuntimeError("This setup expects a Z-up stage.")

    camera_path = "/iw_hub_ROS/chassis/front_hawk/left/camera_left"
    car_path = "/World/sedan_car"
    plate_path = car_path + "/ExperimentRearPlate/Face"
    camera = stage.GetPrimAtPath(camera_path)
    car = stage.GetPrimAtPath(car_path)
    plate = stage.GetPrimAtPath(plate_path)
    if not (camera and camera.IsA(UsdGeom.Camera) and car and plate):
        raise RuntimeError("Load the iw.hub, sedan, and ExperimentRearPlate first.")

    cache = UsdGeom.XformCache()
    parent_world = cache.GetLocalToWorldTransform(car.GetParent())
    if not Gf.IsClose(parent_world, Gf.Matrix4d(1), 1e-8):
        raise RuntimeError("The car's /World parent must have an identity transform.")
    rotate = car.GetAttribute("xformOp:rotateXYZ")
    translate = car.GetAttribute("xformOp:translate")
    rotation = rotate.Get() if rotate else None
    position = translate.Get() if translate else None
    if rotation is None or position is None or abs(rotation[0]) + abs(rotation[1]) > 1e-6:
        raise RuntimeError("Expected the sedan's existing translate and upright rotateXYZ operations.")

    camera_matrix = cache.GetLocalToWorldTransform(camera)
    camera_position = camera_matrix.ExtractTranslation()
    forward = camera_matrix.TransformDir(Gf.Vec3d(0, 0, -1))
    heading = Gf.Vec3d(forward[0], forward[1], 0)
    if heading.GetLength() < 0.5 * forward.GetLength():
        raise RuntimeError("The camera is pointing mostly up/down; inspect its mount first.")
    heading.Normalize()
    plate_matrix = cache.GetLocalToWorldTransform(plate)
    normal = plate_matrix.TransformDir(Gf.Vec3d(0, 0, 1))
    if math.hypot(normal[0], normal[1]) < 1e-6:
        raise RuntimeError("The plate has no usable horizontal facing direction.")
    target = camera_position + heading * distance_m
    target[2] = plate_matrix.ExtractTranslation()[2]
    delta = math.atan2(-heading[1], -heading[0]) - math.atan2(normal[1], normal[0])
    yaw = (rotation[2] + math.degrees(delta) + 180.0) % 360.0 - 180.0

    with Usd.EditContext(stage, stage.GetRootLayer()):
        rotate.Set(Gf.Vec3f(0, 0, yaw))
        rotated_center = UsdGeom.XformCache().GetLocalToWorldTransform(plate).ExtractTranslation()
        translate.Set(Gf.Vec3d(position) + target - rotated_center)

    height_difference = target[2] - camera_position[2]
    return {
        "camera": camera_path,
        "camera_height_m": camera_position[2],
        "plate_height_m": target[2],
        "horizontal_distance_m": distance_m,
        "plate_elevation_deg": math.degrees(math.atan2(height_difference, distance_m)),
    }


if __name__ == "<run_path>" or __name__ == "__main__":
    import omni.timeline
    import omni.usd

    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError("Press Stop before arranging the test scene.")
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Open the vehicle scene first.")
    result = setup_camera_check(stage)
    print("Camera check fixture is ready; only the parked car was repositioned.")
    for key, value in result.items():
        print(f"{key}: {value:.3f}" if isinstance(value, float) else f"{key}: {value}")
    print("Press Play, then view /front_stereo_camera/left/image_raw in rqt_image_view.")
