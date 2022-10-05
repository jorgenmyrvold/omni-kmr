# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio

import carb
import omni
import omni.ui as ui
import omni.kit.commands
import omni.graph.core as og
from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
from omni.isaac.range_sensor._range_sensor import acquire_lidar_sensor_interface
from pxr import Sdf, Gf, UsdPhysics


EXTENSION_NAME = "KMR iiwa importer"
WAREHOUSE_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
KMR_PATH = "/home/jorgen/kmriiwa_description/src/kmriiwa_description/urdf/robot/kmriiwa.urdf"

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[omni.kmr] MyExtension startup")
        disable_extension("omni.isaac.ros_bridge")
        print("ROS Bridge disabled")
        enable_extension("omni.isaac.ros2_bridge")
        print("ROS 2 Bridge enabled")
        
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)
        
        self._window = ui.Window("KMR iiwa importer", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("KUKA KMR iiwa importer and ROS2 initializer")
                ui.Button("Start", clicked_fn=self._on_load_scene)

    def on_shutdown(self):
        self._window = None

    def _on_load_scene(self):
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._create_world(load_stage))

    async def _create_world(self, task):
        done, pending = await asyncio.wait({task})
        if task in done:
            # Create default world prim
            self._stage = omni.usd.get_context().get_stage()
            world_prim = self._stage.OverridePrim("/World")
            self._stage.SetDefaultPrim(world_prim)
            
            # Create a pyhysics scene
            scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
            
            # Create warehouse reference
            ref_warehouse = self._stage.OverridePrim("/World/Warehouse")
            omni.kit.commands.execute("AddReference",
                stage=self._stage,
                prim_path=Sdf.Path("/World/Warehouse"),  # an existing prim to add the reference to.
                reference=Sdf.Reference(WAREHOUSE_PATH)
            )
            
            # Load robot urdf
            res, self._kmr_prim = self._load_kmr()
            self._rig_robot()
            self._setup_graph_kmp()
            self._setup_graph_iiwa()
            self._setup_ros2_graph()
            

    def _load_kmr(self, urdf_filepath=KMR_PATH):
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.make_default_prim = False
        import_config.self_collision = False
        import_config.create_physics_scene = False
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1
        import_config.density = 0.0
        
        result, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_filepath,
            import_config=import_config
        )
        return result, prim_path
    
    def _create_lidar_sensor(self, parent_prim):
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=parent_prim,
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        if result:
            print(f"Created lidar at {prim}")
        else:
            print(f"Failed creating lidar under{parent_prim}")
        return result, prim
    
    def _rig_robot(self):
        result1, self._lidar1_prim = self._create_lidar_sensor(f"{self._kmr_prim}/kmriiwa_laser_B1_link")
        result2, self._lidar2_prim = self._create_lidar_sensor(f"{self._kmr_prim}/kmriiwa_laser_B4_link")
        
        # TODO: Add cameras and other relevant sensors
        
    
    def _setup_lidar_graph(self, lidar_prim):
        # TODO Create generic function for creatig lidar publishers that takes a random lidar
        return
    
    def _setup_ros2_graph(self):
        print("TYPE:",type(self._lidar1_prim))
        print("TYPE:",type(self._stage.GetPrimAtPath(Sdf.Path(self._kmr_prim))))
        print("TYPE:",type(self._stage.GetPrimAtPath(Sdf.Path(f"{self._kmr_prim}/kmriiwa_laser_B1_link/Lidar"))))
        
        
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/ros2_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("isaac_read_lidar_beam_node", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                    ("ros2_context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("constant_string_frame_id", "omni.graph.nodes.ConstantString"),
                    ("isaac_read_sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ros2_publish_laser_scan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                ],
                keys.SET_VALUES: [
                    # TODO: Wrong type passed to "lidarPrim". Fix this later...
                    ("isaac_read_lidar_beam_node.inputs:lidarPrim", self._stage.GetPrimAtPath(Sdf.Path(f"{self._kmr_prim}/kmriiwa_laser_B1_link/Lidar"))),
                    ("ros2_publish_laser_scan.inputs:topicName", "/laser_scan1"),
                    ("constant_string_frame_id.inputs:value", "kmr")
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "isaac_read_lidar_beam_node.inputs:execIn"),
                    ("isaac_read_lidar_beam_node.outputs:execOut", "ros2_publish_laser_scan.inputs:execIn"),
                    ("isaac_read_lidar_beam_node.outputs:azimuthRange", "ros2_publish_laser_scan.inputs:azimuthRange"),
                    ("isaac_read_lidar_beam_node.outputs:depthRange", "ros2_publish_laser_scan.inputs:depthRange"),
                    ("isaac_read_lidar_beam_node.outputs:horizontalFov", "ros2_publish_laser_scan.inputs:horizontalFov"),
                    ("isaac_read_lidar_beam_node.outputs:horizontalResolution", "ros2_publish_laser_scan.inputs:horizontalResolution"),
                    ("isaac_read_lidar_beam_node.outputs:intensitiesData", "ros2_publish_laser_scan.inputs:intensitiesData"),
                    ("isaac_read_lidar_beam_node.outputs:linearDepthData", "ros2_publish_laser_scan.inputs:linearDepthData"),
                    ("isaac_read_lidar_beam_node.outputs:numCols", "ros2_publish_laser_scan.inputs:numCols"),
                    ("isaac_read_lidar_beam_node.outputs:numRows", "ros2_publish_laser_scan.inputs:numRows"),
                    ("isaac_read_lidar_beam_node.outputs:rotationRate", "ros2_publish_laser_scan.inputs:rotationRate"),
                    ("ros2_context.outputs:context", "ros2_publish_laser_scan.inputs:context"),
                    ("constant_string_frame_id.inputs:value", "ros2_publish_laser_scan.inputs:frameId"),
                    ("isaac_read_sim_time.outputs:simulationTime", "ros2_publish_laser_scan.inputs:timeStamp"),
                ],
            }
        )
    
    def _setup_graph_kmp(self):
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/kmp_controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("constant_token_FR", "omni.graph.nodes.ConstantToken"), # Front Right etc...
                    ("constant_token_FL", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_BR", "omni.graph.nodes.ConstantToken"),
                    ("constant_token_BL", "omni.graph.nodes.ConstantToken"),
                    ("constant_double_FR", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_FL", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_BR", "omni.graph.nodes.ConstantDouble"),
                    ("constant_double_BL", "omni.graph.nodes.ConstantDouble"),
                    ("make_array_joint_names", "omni.graph.nodes.MakeArray"),
                    ("make_array_joint_vel", "omni.graph.nodes.MakeArray"),
                    ("articulation_controller", "omni.isaac.core_nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("constant_token_FR.inputs:value", "kmriiwa_back_left_wheel_joint"),
                    ("constant_token_FL.inputs:value", "kmriiwa_back_right_wheel_joint"),
                    ("constant_token_BR.inputs:value", "kmriiwa_front_left_wheel_joint"),
                    ("constant_token_BL.inputs:value", "kmriiwa_front_right_wheel_joint"),  # forward | sideways
                    ("constant_double_FR.inputs:value", -5.0),                               # 5       | -5
                    ("constant_double_FL.inputs:value", 5.0),                               # 5       | 5
                    ("constant_double_BR.inputs:value", 5.0),                               # 5       | 5
                    ("constant_double_BL.inputs:value", -5.0),                               # 5       | -5
                    ("make_array_joint_names.inputs:arraySize", 4),
                    ("make_array_joint_vel.inputs:arraySize", 4),
                    ("articulation_controller.inputs:robotPath", self._kmr_prim),
                    ("articulation_controller.inputs:usePath", True),
                ],
                keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
                    ("constant_token_FR.inputs:value", "make_array_joint_names.inputs:a"),
                    ("constant_token_FL.inputs:value", "make_array_joint_names.inputs:b"),
                    ("constant_token_BR.inputs:value", "make_array_joint_names.inputs:c"),
                    ("constant_token_BL.inputs:value", "make_array_joint_names.inputs:d"),
                    ("constant_double_FR.inputs:value", "make_array_joint_vel.inputs:a"),
                    ("constant_double_FL.inputs:value", "make_array_joint_vel.inputs:b"),
                    ("constant_double_BR.inputs:value", "make_array_joint_vel.inputs:c"),
                    ("constant_double_BL.inputs:value", "make_array_joint_vel.inputs:d"),
                    ("make_array_joint_names.outputs:array", "articulation_controller.inputs:jointNames"),
                    ("make_array_joint_vel.outputs:array", "articulation_controller.inputs:velocityCommand"),
                ]
            }
        )
    
    
    def _setup_graph_iiwa(self):
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/iiwa_controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    
                ],
                keys.SET_VALUES: [
                    
                ],
                keys.CONNECT: [
                    
                ]
            }
        )
    