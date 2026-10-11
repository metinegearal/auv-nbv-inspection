import numpy as np
from std_msgs.msg import Float32MultiArray

class RosTrajectoryOutput:
    def __init__(self, publisher):
        self.publisher = publisher
    def encode(self, trajectory):
        wire = np.asarray(trajectory, dtype=np.float32)
        return wire, wire.astype(np.float64)
    def publish(self, values):
        message = Float32MultiArray()
        message.data = list(values)
        self.publisher.publish(message)
