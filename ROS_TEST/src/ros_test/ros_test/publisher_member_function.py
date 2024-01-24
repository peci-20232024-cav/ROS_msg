# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String

from autoware_auto_control_msgs.msg import AckermannControlCommand 

# define a new QoS profile
test_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', test_profile)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = AckermannControlCommand()
        msg.longitudinal.acceleration = -10.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.longitudinal.acceleration)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
