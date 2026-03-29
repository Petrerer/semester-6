import rclpy
from fs_msgs.msg import ControlCommand
import math

rclpy.init()
node = rclpy.create_node('n')
pub = node.create_publisher(ControlCommand, '/fsds/control_command', 10)
t = 0.0
elapsed = 0.0
dt = 0.05

def cb():
    global t, elapsed
    msg = ControlCommand()
    if elapsed >= 13.0:
        msg.throttle = 0.0
        msg.steering = 0.0
        msg.brake = 1.0
    else:
        msg.throttle = (math.sin(t) + 1) / 2 * 0.3
        msg.steering = 0.0
        msg.brake = 0.0
        t = (t + 0.15) % (2 * math.pi)
        elapsed += dt
    pub.publish(msg)

node.create_timer(dt, cb)
rclpy.spin(node)
