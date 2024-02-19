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

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from math import pi


CENTRE = 180
ANGLE = 20

MIN_FRONT = CENTRE - ANGLE
MAX_FRONT = CENTRE + ANGLE

MAX_LEFT = ANGLE
MAX_RIGHT = CENTRE + 1

OFFSET_LEFT = CENTRE - ANGLE
OFFSET_RIGHT = CENTRE + 1

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #
        
        range_values = ranges[MIN_FRONT : MAX_FRONT]
        left_range_values   = range_values[0 : MAX_LEFT]
        right_range_values   = range_values[MAX_RIGHT : ]
        
        distance_front = min(range_values)
        distance_left = min(left_range_values)
        distance_right = min(right_range_values)
        min_distance = 0.3
        # max_distance = 1.1
        max_distance = 2

        print(ranges[CENTRE])
        vx = 0.0
        wz = 0.0
        # if distance_front >= min_distance and distance_front < max_distance:
        if distance_front < max_distance:
                vx =  distance_front - min_distance
                # if distance_front < ((min_distance + max_distance) / 2): vx = 0.1
                # else: vx = 0.2
                if distance_left < ranges[CENTRE] and distance_left < distance_right:
                        #wz = 0.3
                        pos = OFFSET_LEFT + left_range_values.index(distance_left)
                elif distance_right < ranges[CENTRE] and distance_right < distance_left:
                        #wz = -0.3
                        pos = OFFSET_RIGHT + right_range_values.index(distance_right)
                else: 
                       #wz = 0.0
                       pos = CENTRE
                       
                wz = (pos - CENTRE) / pi

        else:
                vx = 0.0
                wz = 0.0
        #
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
