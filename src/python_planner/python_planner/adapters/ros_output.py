import numpy as np
from std_msgs.msg import Float32MultiArray, Bool

class RosTrajectoryOutput:
    def __init__(self, publisher):
        self.publisher = publisher
    def encode(self, trajectory, camera_offset):
        wire = np.asarray(trajectory, dtype=np.float64).copy()
        wire[:, :3] += camera_offset
        wire = wire.astype(np.float32)
        executed = wire.astype(np.float64)
        executed[:, :3] -= camera_offset
        return wire, executed
    def publish(self, values):
        message = Float32MultiArray()
        message.data = list(values)
        self.publisher.publish(message)

class RosCompletionOutput:
    def __init__(self, publisher):
        self.publisher = publisher
    def publish(self, finished):
        self.publisher.publish(Bool(data=finished))
