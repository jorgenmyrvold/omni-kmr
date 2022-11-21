import omni.ext
import omni.ui as ui
import omni
from omni.isaac.core import World
from pxr import UsdGeom, Gf
import asyncio
from omni.isaac.core.utils.stage import create_new_stage_async


class MyExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[omni.plot_pose] Pose plotter startup")

        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}

        self.logger_is_active = False

        self._window = ui.Window("Pose plotter", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label('Click "Start Logger" to initialize the logging')
                with ui.HStack():
                    ui.Button("Start Logger", clicked_fn=self.init_extention)
                    ui.Button("Stop Logger", clicked_fn=self.stop_logger)


    def init_extention(self):
        async def init_extention_async():
            if World.instance() is None:
                await create_new_stage_async()
                self._world = World(**self._world_settings)
                await self._world.initialize_simulation_context_async()
                self.setup_scene()
            else:
                self._world = World.instance()

            await self.setup_post_load()
            self.start_logger()
        
        asyncio.ensure_future(init_extention_async())
        return


    async def setup_post_load(self):
        self._world = World.instance()
        print("type world", type(self._world))
        print("world instance", World.instance)
        self._cube = self._world.scene.get_object("base_link")
        self._world.add_physics_callback("sim_step", callback_fn=self.log_step)


    def start_logger(self):
        self.target_prim = None
        self.logger_is_active = True
        print("[omni.plot_pose] Plotter is activated")


    def stop_logger(self):
        self.logger_is_active = False
        print("[omni.plot_pose] Plotter is deactivated")


    def log_step(self):
        if self.logger_is_active:
            self.print_cube_info()
            self.print_current_pose()
            print("logging!")


    # From: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html
    def print_cube_info(self):
        position, orientation = self._cube.get_world_pose()
        linear_velocity = self._cube.get_linear_velocity()
        # will be shown on terminal
        print("Cube position is : " + str(position))
        print("Cube's orientation is : " + str(orientation))
        print("Cube's linear velocity is : " + str(linear_velocity))


    # From: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_python_snippets.html#get-world-transform-at-current-timestamp-for-selected-prims
    def print_current_pose(self):
        # Get list of selected primitives
        selected_prims = self.usd_context.get_selection().get_selected_prim_paths()
        # Get the current timecode
        timecode = self.timeline.get_current_time() * self.timeline.get_time_codes_per_seconds()
        # Loop through all prims and print their transforms
        for prim in selected_prims:
            curr_prim = self.stage.GetPrimAtPath(prim)
            print("Selected", prim)
            pose = omni.usd.utils.get_world_transform_matrix(curr_prim, timecode)
            print("Matrix Form:", pose)
            print("Translation: ", pose.ExtractTranslation())
            q = pose.ExtractRotation().GetQuaternion()
            print(
                "Rotation: ", q.GetReal(), ",", q.GetImaginary()[0], ",", q.GetImaginary()[1], ",", q.GetImaginary()[2]
            )


    def on_shutdown(self):
        print("[omni.plot_pose] MyExtension shutdown")
