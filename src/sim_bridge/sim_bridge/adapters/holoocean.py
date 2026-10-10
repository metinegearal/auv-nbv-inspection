from contextlib import contextmanager
import json

def load_scenario(path):
    with open(path, 'r') as stream:
        return json.load(stream)

class HoloOceanSessions:
    def __init__(self, make, scenario):
        self.make = make
        self.scenario = scenario
    @contextmanager
    def open(self):
        with self.make(scenario_cfg=self.scenario) as environment:
            yield HoloOceanSession(environment)

class HoloOceanSession:
    def __init__(self, environment):
        self.environment = environment
    def step(self, command):
        return self.environment.step(list(command.values))
    def set_ocean_currents(self, agent, velocity):
        return self.environment.set_ocean_currents(agent, velocity)
