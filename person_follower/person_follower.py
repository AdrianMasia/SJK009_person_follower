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
ANGLE = 30

MIN_FRONT = CENTRE - ANGLE  # 150
MAX_FRONT = CENTRE + ANGLE  # 210

MIN_DISTANCE = 0.55
MAX_DISTANCE = 1.75
MAX_VEL = 6.67

VEL_SMOOTH_FACTOR = 0.7
ANGLE_SMOOTH_FACTOR = 0.3

# Lo de la derivada y la integral, o se ponen valores muy bajos, o molestan
DERIVATE_SMOOTH_FACTOR = 0.005
INTEGRAL_SMOOTH_FACTOR = 0.00005

class PersonFollower(Node):

    prev_angle_error = 0.0
    angle_error_acumulation = 0.0

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

        # El conjunto de valores tal que: (indice de [0, ANGLE*2] , distancia)
        range_values = list(enumerate(ranges[MIN_FRONT : MAX_FRONT]))
        # Conjunto ordenado por la distancia (x es la tupla: x[0] indice, x[1] distanca)
        range_values.sort(key= lambda x: x[1])

        # Distancia menor y su índice.
        #  El índice es importante porque nos ayuda a saber el ángulo
        index, distance = range_values[0]

        # Avanza si está entre dos umbrales de distancia
        if MIN_DISTANCE < distance < MAX_DISTANCE:
            # Idea: a más distancia, más velocidad (pero con límites)
            vx = min(distance - MIN_DISTANCE, MAX_VEL)

            # El índice estará entre 0 y 2·Ángulo  (o angulo_izq + angulo_der)
            #   Por lo tanto, desplazamos el indice para que sea el ángulo real.
            angle = index + MIN_FRONT

            angle_error = CENTRE - angle

            # Pasamos la diferencia de ángulos a radianes
            wz = angle_error / pi

            self.prev_angle_error += angle_error
            self.angle_error_acumulation = 0.0 if angle_error == 0 else (self.angle_error_acumulation + angle_error)

            # Aplicamos las constantes para suavizar las velocidades líneales y angulares (respectivamente)
            vx *= VEL_SMOOTH_FACTOR
            wz = (wz * ANGLE_SMOOTH_FACTOR +
                  self.prev_angle_error * DERIVATE_SMOOTH_FACTOR +
                  self.angle_error_acumulation * INTEGRAL_SMOOTH_FACTOR)

            print(f"vx: {vx} |  wz: {wz} | angle_min: {angle_min}")
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
