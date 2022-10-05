# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
import math
import omni.kit.commands
import asyncio
import weakref
import omni.ui as ui
import omni.graph.core as og

from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.urdf import _urdf

from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder

from pxr import UsdLux, Sdf, Gf, UsdPhysics

EXTENSION_NAME = "KMR iiwa importer"
WAREHOUSE_PATH = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
KMR_PATH = "/home/jorgen/kmriiwa_description/src/kmriiwa_description/urdf/robot/kmriiwa.urdf"

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        print("[omni.kmr] MyExtension startup")

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
            self._setup_graph_kmp()
            self._setup_graph_iiwa()
            

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
    