"""Add an elevated 2D RTX LiDAR to the iw.hub camera-mast test scene.

Run in Isaac Sim 5.1's Script Editor while stopped, after Save As. The new
sensor references the existing front sensor's profile and follows the chassis.
Its simple housing/support are visual geometry only, with no mass or colliders.
Nothing is saved automatically. Existing low LiDARs and camera poses stay intact.
"""

import math

from pxr import Gf, Sdf, Usd, UsdGeom


CHASSIS = "/iw_hub_ROS/chassis"
ASSEMBLY = CHASSIS + "/ObservationLidar"
SENSOR = ASSEMBLY + "/observation_lidar"
SOURCE = CHASSIS + "/Lidar_Front/RPLidar_S2E"
MAST = CHASSIS + "/ExperimentCameraMast"
GRAPH = "/iw_hub_ROS/observation_lidar_ros"
TF_NODE = "/iw_hub_ROS/transform_tree_odometry/tf_tree_base_link_to_sensors"
OWNER = "amr_observation_lidar"
TOPIC = "/observation_lidar/scan"
FRAME = "observation_lidar"

NODES = [
    ("Tick", "omni.graph.action.OnPlaybackTick"),
    ("Once", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("Render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
    ("Scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
    ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
]
CONNECTIONS = [
    ("Tick.outputs:tick", "Once.inputs:execIn"),
    ("Once.outputs:step", "Render.inputs:execIn"),
    ("Render.outputs:execOut", "Scan.inputs:execIn"),
    ("Render.outputs:renderProductPath", "Scan.inputs:renderProductPath"),
    ("Context.outputs:context", "Scan.inputs:context"),
    ("Tick.outputs:tick", "TF.inputs:execIn"),
    ("Context.outputs:context", "TF.inputs:context"),
    ("Time.outputs:simulationTime", "TF.inputs:timeStamp"),
]


def prepare_sensor(stage, height_m=0.80):
    """Author the sensor and its Xform frame; usable without starting Kit."""
    if not math.isfinite(height_m) or not 0.70 <= height_m <= 1.20:
        raise ValueError("Choose a scan-plane height between 0.70 and 1.20 m.")
    if (not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0)
            or UsdGeom.GetStageUpAxis(stage) != "Z"):
        raise RuntimeError("Expected a meter-scale, Z-up stage.")
    chassis = stage.GetPrimAtPath(CHASSIS)
    source = stage.GetPrimAtPath(SOURCE)
    camera_mount = stage.GetPrimAtPath(CHASSIS + "/front_hawk")
    post = stage.GetPrimAtPath(MAST + "/Post")
    top = stage.GetPrimAtPath(MAST + "/Top")
    tf_node = stage.GetPrimAtPath(TF_NODE)
    if not all((chassis, source, camera_mount, post, top, tf_node)):
        raise RuntimeError("Open the iw.hub mast scene. Run raise_camera_mast.py first if needed.")
    if source.GetTypeName() != "OmniLidar":
        raise RuntimeError("Expected the Isaac Sim 5.1 OmniLidar at " + SOURCE)
    for name in ("elevationDeg", "vertOffsetM"):
        values = source.GetAttribute("omni:sensor:Core:emitterState:s001:" + name).Get()
        if values is None or len(values) != 1 or abs(values[0]) > 1e-6:
            raise RuntimeError("Expected the existing single, horizontal 2D scan profile.")
    if tf_node.GetAttribute("node:type").Get() != "isaacsim.ros2.bridge.ROS2PublishTransformTree":
        raise RuntimeError("The existing sensor TF publisher has changed; inspect it first.")
    targets = tf_node.GetRelationship("inputs:targetPrims")
    parents = tf_node.GetRelationship("inputs:parentPrim").GetTargets()
    if not targets or parents != [Sdf.Path(CHASSIS + "/base_link")]:
        raise RuntimeError("Expected the existing base_link-to-sensors TF publisher.")
    for path in (ASSEMBLY, GRAPH):
        existing = stage.GetPrimAtPath(path)
        if existing and existing.GetCustomDataByKey("fixtureOwner") != OWNER:
            raise RuntimeError("An unrelated prim already occupies " + path)

    cache = UsdGeom.XformCache()
    chassis_world = cache.GetLocalToWorldTransform(chassis)
    for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)):
        direction = chassis_world.TransformDir(axis)
        if not math.isclose(direction.GetLength(), 1.0, abs_tol=1e-5):
            raise RuntimeError("Expected an unscaled chassis.")
    if not Gf.IsClose(chassis_world.TransformDir(Gf.Vec3d(0, 0, 1)), Gf.Vec3d(0, 0, 1), 1e-5):
        raise RuntimeError("Stop the simulation with the chassis upright before adding the sensor.")

    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    camera_top = bounds.ComputeWorldBound(camera_mount).ComputeAlignedRange().GetMax()[2]
    mast_top = bounds.ComputeWorldBound(top).ComputeAlignedRange().GetMax()[2]
    # The housing ends 1 cm below the scan plane; its bottom is 3.5 cm below.
    if height_m - 0.035 < camera_top + 0.003:
        raise RuntimeError("The LiDAR housing would overlap the camera height; choose a higher scan plane.")
    if height_m - 0.035 <= mast_top:
        raise RuntimeError("The scan plane is too low for the current mast.")
    post_position = cache.GetLocalToWorldTransform(post).ExtractTranslation()
    position = chassis_world.GetInverse().Transform(Gf.Vec3d(post_position[0], post_position[1], height_m))
    support_bottom = mast_top - height_m
    support_top = -0.035

    with Usd.EditContext(stage, stage.GetRootLayer()):
        assembly = UsdGeom.Xform.Define(stage, ASSEMBLY)
        assembly.GetPrim().SetCustomDataByKey("fixtureOwner", OWNER)
        assembly.GetPrim().SetCustomDataByKey("purpose", "Visual housing/support only; no added mass or colliders")
        # Publish the ordinary Xform at the optical origin, rather than asking
        # the existing TF publisher to resolve an additional OmniLidar prim.
        assembly.GetPrim().CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(FRAME)
        assembly.MakeMatrixXform().Set(Gf.Matrix4d(1).SetTranslate(position))
        sensor = stage.DefinePrim(SENSOR, "OmniLidar")
        # Internal reference reuses the same installed profile, without copying
        # downloaded NVIDIA assets into the repository. Ancestor poses are not inherited.
        sensor.GetReferences().SetReferences([Sdf.Reference(primPath=Sdf.Path(SOURCE))])
        UsdGeom.Xformable(sensor).MakeMatrixXform().Set(Gf.Matrix4d(1))
        sensor.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(FRAME)
        UsdGeom.Imageable(sensor).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)

        housing = UsdGeom.Cylinder.Define(stage, ASSEMBLY + "/Housing")
        housing.CreateAxisAttr("Z")
        housing.CreateRadiusAttr(0.04)
        housing.CreateHeightAttr(0.025)
        housing.MakeMatrixXform().Set(Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(0, 0, -0.0225)))
        housing.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.14)])
        support = UsdGeom.Cube.Define(stage, ASSEMBLY + "/Support")
        support.CreateSizeAttr(1.0)
        matrix = Gf.Matrix4d(1).SetScale(Gf.Vec3d(0.025, 0.025, support_top - support_bottom))
        matrix.SetTranslateOnly(Gf.Vec3d(0, 0, (support_bottom + support_top) / 2))
        support.MakeMatrixXform().Set(matrix)
        support.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.10, 0.12)])

    return {"scan_height_m": height_m, "camera_top_m": camera_top,
            "camera_clearance_m": height_m - camera_top,
            "topic": TOPIC, "frame": FRAME}


def connect_ros(stage):
    """Create or repair the persistent scan and dedicated sensor TF graph."""
    import omni.graph.core as og
    import usdrt

    graph = stage.GetPrimAtPath(GRAPH)
    missing_nodes = []
    if graph:
        if graph.GetCustomDataByKey("fixtureOwner") != OWNER:
            raise RuntimeError("An unrelated graph occupies " + GRAPH)
    for name, node_type in NODES:
        node = stage.GetPrimAtPath(GRAPH + "/" + name)
        if not node:
            missing_nodes.append((name, node_type))
        elif node.GetAttribute("node:type").Get() != node_type:
            raise RuntimeError("The observation graph was edited; inspect node " + name)
    # Match the existing low-LiDAR DDS context and simulation-time behavior.
    context = stage.GetPrimAtPath("/iw_hub_ROS/ros_lidars/ros2_context")
    original = stage.GetPrimAtPath("/iw_hub_ROS/ros_lidars/publish_front_2d_lidar_scan")
    original_time = stage.GetPrimAtPath("/iw_hub_ROS/transform_tree_odometry/isaac_read_simulation_time")

    def value(prim, name, default):
        attr = prim.GetAttribute(name) if prim else None
        result = attr.Get() if attr else None
        return default if result is None else result

    values = [
        ("Context.inputs:domain_id", value(context, "inputs:domain_id", 0)),
        ("Context.inputs:useDomainIDEnvVar", value(context, "inputs:useDomainIDEnvVar", True)),
        ("Render.inputs:width", 1), ("Render.inputs:height", 1),
        ("Scan.inputs:frameId", FRAME), ("Scan.inputs:topicName", TOPIC),
        ("Scan.inputs:type", "laser_scan"), ("Scan.inputs:showDebugView", False),
        ("Scan.inputs:useSystemTime", value(original, "inputs:useSystemTime", False)),
        ("Scan.inputs:resetSimulationTimeOnStop", value(original, "inputs:resetSimulationTimeOnStop", True)),
        ("Time.inputs:resetOnStop", value(original_time, "inputs:resetOnStop", True)),
        ("TF.inputs:topicName", "/tf"), ("TF.inputs:staticPublisher", False),
        ("TF.inputs:parentPrim", [usdrt.Sdf.Path(CHASSIS + "/base_link")]),
        ("TF.inputs:targetPrims", [usdrt.Sdf.Path(ASSEMBLY)]),
        ("Render.inputs:cameraPrim", [usdrt.Sdf.Path(SENSOR)]),
    ]
    # Controller.edit only maps relative port names for nodes created in THAT
    # call. Existing nodes have no short-name mapping during repair/reruns.
    # Qualify both SET_VALUES and CONNECT ports so mixed old/new graphs work.
    edits = {og.Controller.Keys.SET_VALUES: [(GRAPH + "/" + port, val) for port, val in values]}
    if missing_nodes:
        edits[og.Controller.Keys.CREATE_NODES] = missing_nodes
    missing_connections = []
    for source, destination in CONNECTIONS:
        destination_attr = stage.GetAttributeAtPath(GRAPH + "/" + destination)
        connected = destination_attr.GetConnections() if destination_attr else []
        if Sdf.Path(GRAPH + "/" + source) not in connected:
            missing_connections.append((GRAPH + "/" + source, GRAPH + "/" + destination))
    if missing_connections:
        edits[og.Controller.Keys.CONNECT] = missing_connections
    with Usd.EditContext(stage, stage.GetRootLayer()):
        # A dictionary ALWAYS requests creation in OmniGraph 1.141. Use the
        # existing path for edits, including upgrades of the earlier 5-node graph.
        graph_spec = GRAPH if graph else {"graph_path": GRAPH, "evaluator_name": "execution"}
        og.Controller.edit(graph_spec, edits)
        stage.GetPrimAtPath(GRAPH).SetCustomDataByKey("fixtureOwner", OWNER)
        # Migrate the first version's target out of the shared TF publisher.
        # Keep every original robot/camera target and update OmniGraph as well as USD.
        targets = stage.GetPrimAtPath(TF_NODE).GetRelationship("inputs:targetPrims").GetTargets()
        if Sdf.Path(SENSOR) in targets:
            og.Controller.set(og.Controller.attribute(TF_NODE + ".inputs:targetPrims"),
                              [usdrt.Sdf.Path(str(p)) for p in targets if p != Sdf.Path(SENSOR)])


if __name__ == "<run_path>" or __name__ == "__main__":
    import omni.timeline
    import omni.usd

    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError("Press Stop before adding the observation LiDAR.")
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Open the iw.hub camera-mast scene first.")
    result = prepare_sensor(stage)
    connect_ros(stage)
    for key, val in result.items():
        print(f"{key}: {val:.3f}" if isinstance(val, float) else f"{key}: {val}")
    print("Observation LiDAR added. Save, press Play, and allow a full scan to accumulate.")
    print("TF configured: base_link -> observation_lidar (dedicated TF publisher).")
    print("RViz: Fixed Frame = observation_lidar; LaserScan = /observation_lidar/scan; Best Effort.")
