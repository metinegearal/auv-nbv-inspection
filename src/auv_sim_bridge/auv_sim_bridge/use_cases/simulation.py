"""Simulation stepping policy; no simulator, filesystem or ROS imports."""
import traceback
from auv_contracts.ports import Diagnostics
from ..domain.simulation import SessionFactory, SensorDispatcher

class SimulationRunner:
    def __init__(self, sessions: SessionFactory, sensors: SensorDispatcher,
                 commands, pump, diagnostics: Diagnostics, step_limit=None):
        self.sessions = sessions
        self.sensors = sensors
        self.commands = commands
        self.pump = pump
        self.diagnostics = diagnostics
        self.step_limit = step_limit
        self._running = True

    def run(self):
        """
        Run HoloOcean loop inside context manager. Blocking until finished or step_limit reached.
        """
        self.diagnostics.info("Entering holoocean.make context...")
        steps = 0
        with self.sessions.open() as env:
            self.diagnostics.info("HoloOcean env created.")
            try:
                while self._running:
                    if self.step_limit is not None and steps >= self.step_limit:
                        self.diagnostics.info("Reached step limit (%d)." % self.step_limit)
                        break
                    state = env.step(self.commands())
                    steps += 1

                    self.sensors.dispatch(state, env)

                    # small sleep is not necessary — env.step controls sim time — but let ROS spin once
                    self.pump()

            except KeyboardInterrupt:
                self.diagnostics.info('KeyboardInterrupt - exiting holoocean loop.')
            except Exception as e:
                self.diagnostics.error("Exception inside HoloOcean main loop: %s" % str(e))
                self.diagnostics.error(traceback.format_exc())
            finally:
                self.diagnostics.info("HoloOcean context exited after %d steps." % steps)

