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

from autoware_auto_perception_msgs.msg import TrafficSignalArray, TrafficSignal, TrafficLight

# define a new QoS profile
test_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(TrafficSignalArray, '/perception/traffic_light_recognition/traffic_signals', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #insert code for receiving from camera here
        #receive dict of elements per signal
        camera = {1:[[1,16,1,0.9],[3,5,3,0.9],[2,7,2,0.9]],
                  2:[[1,16,2,0.9],[3,5,1,0.9],[2,8,3,0.9]],
                  3:[[1,16,3,0.9],[3,5,2,0.9],[2,9,1,0.9]]}
        signals = []
        for id in camera.keys():
            lights = []
            for elemento in camera[id]:
                light = TrafficLight()
                light.color = elemento[0]
                light.shape = elemento[1]
                light.status = elemento[2]
                light.confidence = elemento[3]
            lights+= [light]
            signal = set_signal(id,lights)
            signals.append(signal)
        msg = set_signal_array(signals)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.signals)
        self.i += 1

#function to return a signal
def set_signal(id, lights):
    signal = TrafficSignal()
    signal.map_primitive_id = id
    signal.lights = lights
    return signal

#function to return the signal array
def set_signal_array(signals):
    signal_array = TrafficSignalArray()
    signal_array.signals = signals
    return signal_array

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
