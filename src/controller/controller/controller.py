import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import select, termios, sys, tty

from autoware_auto_control_msgs.msg import AckermannControlCommand 

# global variable to toggle control
controlToggle = False

# define a new QoS profile
test_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

def getKey():
    counter = 1
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    x = ' '
    while True:
            counter += 1
            if select.select([sys.stdin,],[],[],0.1)[0]:
                x=sys.stdin.read(1)[0]
            else:
                x = ' '
            return x

class PubSubNode(Node):

    def __init__(self):
        print("""Controls:
              w: accelerate
              a: steer left
              s: slow down
              d: steer right
              e: emergency stop
              i: toggle control""")
        super().__init__("pub_sub_node")

        self.publisher = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", test_profile)
        self.subscriber = self.create_subscription(AckermannControlCommand, "/control/command/control_cmd_diff", self.listener_callback, 10)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        #print("I heard: ", msg)
        global controlToggle
        control = getKey()
        if control == 'i':
            controlToggle = not controlToggle
            print("Manual mode: ", controlToggle)
        if not controlToggle:
            self.publisher.publish(msg)
        else:
            control = getKey()
            speed = msg.longitudinal.speed
            msg = AckermannControlCommand()
            if control == 'w':
                msg.longitudinal.acceleration = 5.0
                msg.longitudinal.speed = speed
                print("Accelerating")
            elif control == 'a':
                msg.lateral.steering_tire_angle = 0.1
                msg.longitudinal.speed = speed
                print("Steering left")
            elif control == 's':
                msg.longitudinal.acceleration = -5.0
                msg.longitudinal.speed = speed
                print("Slowing down")
            elif control == 'd':
                msg.lateral.steering_tire_angle = -0.1
                msg.longitudinal.speed = speed
                print("Steering right")
            elif control == 'e':
                msg.longitudinal.speed = 0.0
                msg.longitudinal.acceleration = -200.0
                msg.lateral.steering_tire_angle = 0.0
                print("Emergency stop")
            self.publisher.publish(msg)



def main(args=None):

    rclpy.init(args=args)
    pub_sub_node = PubSubNode()
    rclpy.spin(pub_sub_node)
    pub_sub_node.destroy_node()
    rclpy.shutdown()