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

MIN_FRONT = CENTRE - ANGLE  # 160
MAX_FRONT = CENTRE + ANGLE  # 200

MIN_LEFT = CENTRE       # 180
MAX_LEFT = MAX_FRONT    #

MIN_RIGHT = MIN_FRONT
MAX_RIGHT = CENTRE - 1

OFFSET_LEFT = MIN_LEFT
OFFSET_RIGHT = MAX_LEFT

MIN_DISTANCE = 0.3
# max_distance = 1.1
MAX_DISTANCE = 1.5

MAX_VEL = 6.67

ALPHA = 0.5 # TODO: Buscar un nombre mejor
NUM_DECIMALS = 4

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

        # Obtenemos los conjuntos de los valores que consideramos delante

        # El conjunto de valores
        range_values = ranges[MIN_FRONT : MAX_FRONT]
        # Los valores de lo que consideramos que es la derecha
        left_range_values = ranges[MIN_LEFT : MAX_LEFT]
        # Los valores de lo que conisderamos que es la izquierda
        right_range_values = ranges[MIN_RIGHT : MAX_RIGHT]

        # La distancia menor de los valores
        distance_front = round(min(range_values), NUM_DECIMALS)
        # La distancia menor del conjunto de la izquierda
        distance_left = round(min(left_range_values), NUM_DECIMALS)
        # La distancia menor del conjunto de la derecha
        distance_right = round(min(right_range_values), NUM_DECIMALS)
        # La distancia en el centro
        distance_centre = round(ranges[CENTRE], NUM_DECIMALS)

        # print(ranges[CENTRE])

        if distance_front < MAX_DISTANCE:
            # Idea: a más distancia, más velocidad.
            vx = min(distance_front - MIN_DISTANCE, MAX_VEL)
            print(f"vx: {vx} | distance_front: {distance_front}")
            print(f"distance_left: {distance_left} | distance_centre: {distance_centre} | distance_right: {distance_right} ")
            if distance_left < distance_centre and distance_left < distance_right:
                # Avanza izquierda
                print("Avanza izquierda")
                pos = left_range_values.index(distance_left)

                print(f"pos: {pos}")

            elif distance_right < distance_centre and distance_right < distance_left:
                # Avanza derecha
                print("Avanza izquierda")
                pos = right_range_values.index(distance_right)

                print(f"pos: {pos}")
            else:
                # Avanza de frente.
                print("Avanza de frente")
                #wz = 0.0
                pos = CENTRE

            wz = (CENTRE - pos) * ALPHA / pi

            print(f"wz: {wz} | pos: {pos}")
            print("--------")
        else:
                vx = 0.0
                wz = 0.0

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
