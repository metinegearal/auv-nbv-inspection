"""Sensor decoding registry, separate from simulation scheduling."""
import traceback
import numpy as np

class SensorDispatch:
    def __init__(self, outputs, diagnostics, current_field, agent='auv0'):
        self.outputs = outputs
        self.diagnostics = diagnostics
        self.current_field = current_field
        self.agent = agent
        self.handlers = (self.cameras, self.range_finder, self.dvl, self.imu,
                         self.depth, self.magnetic, self.location)

    def dispatch(self, state, session):
        for handler in self.handlers:
            handler(state, session)

    def cameras(self, state, env):
        try:
            try:
                # Camera: expect HxWxC (RGB)
                cam = state.get("FrontCamera", None) if isinstance(state, dict) else None
                if cam is None:
                    # maybe state itself is an array? try attribute
                    cam = getattr(state, "FrontCamera", None)
                if cam is not None:
                    cam_np = np.array(cam)
                    # If sensor returns HxWx4 RGBA or HxWx3
                    self.outputs._publish_camera(cam_np)
            except Exception:
                self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())

            try:
                # Camera: expect HxWxC (RGB)
                cam = state.get("DownCamera", None) if isinstance(state, dict) else None
                if cam is None:
                    # maybe state itself is an array? try attribute
                    cam = getattr(state, "DownCamera", None)
                if cam is not None:
                    cam_np_2 = np.array(cam)
                    # If sensor returns HxWx4 RGBA or HxWx3
                    self.outputs._publish_camera_down(cam_np_2)
            except Exception:
                self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())

            # depth = compute_stereo_depth(state.get("DownCamera", None), state.get("FrontCamera", None), focal_length_px=256, baseline_meters=0.5)
            # self.outputs._publish_depth(depth)

        except Exception:
            self.diagnostics.debug("camera publish failed: %s" % traceback.format_exc())


    def range_finder(self, state, env):
        try:
            range_finder = state.get("RangeFinder", None) if isinstance(state, dict) else getattr(state, "RangeFinder", None)
            self.outputs._publish_range_finder(range_finder)
        except Exception:
            self.diagnostics.debug("range finder publish failed: %s" % traceback.format_exc())


    def dvl(self, state, env):
        try:
            dvl = state.get("DVLSensor", None) if isinstance(state, dict) else getattr(state, "DVLSensor", None)
            #print(dvl)
            self.outputs._publish_dvl(dvl)
        except Exception:
            self.diagnostics.debug("dvl failed: %s" % traceback.format_exc())


    def imu(self, state, env):
        try:
            imu = state.get("IMUSensor", None) if isinstance(state, dict) else getattr(state, "IMUSensor", None)
            self.outputs._publish_imu(imu)
        except Exception:
            self.diagnostics.debug("imu failed: %s" % traceback.format_exc())


    def depth(self, state, env):
        try:
            depth = state.get("DepthSensor", None) if isinstance(state, dict) else getattr(state, "DepthSensor", None)
            #print(depth)
            self.outputs._publish_depth(depth)
        except Exception:
            self.diagnostics.debug("depth failed: %s" % traceback.format_exc())


    def magnetic(self, state, env):
        try:
            mag = state.get("MagnetometerSensor", None) if isinstance(state, dict) else getattr(state, "MagnetometerSensor", None)
            #print(mag)
            self.outputs._publish_mag(mag)
        except Exception:
            self.diagnostics.debug("mag failed: %s" % traceback.format_exc())


    def location(self, state, env):
        try:
            loc = state.get("LocationSensor", None) if isinstance(state, dict) else getattr(state, "LocationSensor", None)
            current_velocity = self.current_field(loc)
            env.set_ocean_currents(self.agent, current_velocity)
            self.outputs._publish_location(loc)
        except Exception:
            self.diagnostics.debug("location failed: %s" % traceback.format_exc())
