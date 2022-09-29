from ast import Pass
import omni.ext
import omni
import math
import omni.kit.commands
import asyncio
import weakref
import omni.ui as ui
from omni.kit.menu.utils import add_menu_items, remove_menu_items, MenuItemDescription
from omni.isaac.urdf import _urdf
import omni.graph.core as og


from omni.isaac.ui.ui_utils import setup_ui_headers, get_style, btn_builder

from pxr import UsdLux, Sdf, Gf, UsdPhysics


# Functions and vars are available to other extension as usual in python: `example.python_ext.some_public_function(x)`
def some_public_function(x: int):
    print(f"[omni.hello.world] some_public_function was called with {x}")
    return x ** x


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[omni.hello.world] MyExtension startup")

        self._count = 0

        self._window = ui.Window("KMR iiwa importer", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("KUKA KMR iiwa importer and ROS2 initializer")
                ui.Button("Add", clicked_fn=self._on_load_robot)

    def on_shutdown(self):
        print("[omni.hello.world] MyExtension shutdown")
    
    def _on_load_robot(self):
        load_stage = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._load_scene(load_stage)) 

    async def _load_scene(self, task):
        done, pending = await asyncio.wait({task})
        if task in done:
            # status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
            
            await self._load_kmr(self)
            
            viewport = omni.kit.viewport_legacy.get_default_viewport_window()
            viewport.set_camera_position("/OmniverseKit_Persp", 3.00, -3.50, 1.13, True)
            viewport.set_camera_target("/OmniverseKit_Persp", -0.96, 1.08, -0.20, True)
            stage = omni.usd.get_context().get_stage()
            scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
            result, plane_path = omni.kit.commands.execute(
                "AddGroundPlaneCommand",
                stage=stage,
                planePath="/groundPlane",
                axis="Z",
                size=1500.0,
                position=Gf.Vec3f(0, 0, -0.50),
                color=Gf.Vec3f(0.5),
            )
            # make sure the ground plane is under root prim and not robot
            omni.kit.commands.execute(
                "MovePrimCommand", path_from=plane_path, path_to="/groundPlane", keep_world_transform=True
            )
            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)


    async def _load_kmr(self, task):
        # done, pending = await asyncio.wait({task})
        # if task in done:
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = False
        import_config.make_default_prim = False
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        import_config.distance_scale = 1
        import_config.density = 0.0
        # Get the urdf file path
        root_path = "/home/jorgen/kmriiwa_description/src/kmriiwa_description/urdf/robot"
        file_name = "kmriiwa.urdf"
        # Finally import the robot
        result, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", 
                                                          urdf_path="{}/{}".format(root_path, file_name),
                                                          import_config=import_config,)