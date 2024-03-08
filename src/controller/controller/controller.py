import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import select, termios, sys, tty, pygame

from autoware_auto_control_msgs.msg import AckermannControlCommand 

# global variable to toggle control
controlToggle = False

# define a new QoS profile
test_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

def get_keys():
    # counter = 1
    # orig_settings = termios.tcgetattr(sys.stdin)
    # tty.setcbreak(sys.stdin)
    # x = ' '
    # while True:
    #         counter += 1
    #         if select.select([sys.stdin,],[],[],0.1)[0]:
    #             x=sys.stdin.read(1)[0]
    #         else:
    #             x = ' '
    #         return x
    keys = pygame.key.get_pressed()
    return keys
    
def switch_control(keys):
    global controlToggle
    if keys[pygame.K_i]:
        controlToggle = not controlToggle
        if controlToggle:
            print("Manual mode enabled ")
        else:
            print("Manual mode disabled")

def get_input(speed, keys):
    msg = AckermannControlCommand()
    if keys[pygame.K_e]:
        msg.longitudinal.speed = 0.0
        msg.longitudinal.acceleration = -200.0
        msg.lateral.steering_tire_angle = 0.0
    if keys[pygame.K_w]:
        msg.longitudinal.acceleration = 5.0
    if keys[pygame.K_a]:
        msg.lateral.steering_tire_angle = 0.1
    if keys[pygame.K_s]:
        msg.longitudinal.acceleration = -5.0
    if keys[pygame.K_d]:
        msg.lateral.steering_tire_angle = -0.1
    if keys[pygame.K_w] and keys[pygame.K_s]:
        msg.longitudinal.acceleration = 0.0
    if keys[pygame.K_a] and keys[pygame.K_d]:
        msg.longitudinal.acceleration = 0.0
    msg.longitudinal.speed = speed
    return msg

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

        global controlToggle

        pygame.event.pump()  # Process pending events
        keys = get_keys()
        switch_control(keys)
        if not controlToggle:
            self.publisher.publish(msg)
        else:
            speed = msg.longitudinal.speed
            msg = get_input(speed, keys)
            self.publisher.publish(msg)



def main(args=None):

    # pygame setup
    pygame.init()
    screen = pygame.display.set_mode((720, 480))
    clock = pygame.time.Clock()
    running = True
    rclpy.init(args=args)
    pub_sub_node = PubSubNode()
    rclpy.spin(pub_sub_node)
    pub_sub_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()