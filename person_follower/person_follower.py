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

# -- CONSTANTES -- #
CENTRE = 180
ANGLE = 30

# Constante para corregir el error de la detección del ángulo laser.
#LIDAR_ERROR = -100
# Sentido horario (1 sí, -1 no)
#IS_CLOCKWISE = -1

# Centro con el error del LIDAR.
#CENTRE = REAL_CENTRE + LIDAR_ERROR

MIN_FRONT = CENTRE - ANGLE # 150
MAX_FRONT = CENTRE + ANGLE # 210

MIN_DISTANCE = 0.50
MAX_DISTANCE = 2.50
MAX_VEL = 6.67
MIN_VEL = 0.20

VEL_SMOOTH_FACTOR = 0.50 # 0.5
ANGLE_SMOOTH_FACTOR = 0.15 # 0.15

DERIVATE_SMOOTH_FACTOR = 0.00010 # 0.010
INTEGRAL_SMOOTH_FACTOR = 0.0000010 # 0.00010

class PersonFollower(Node):

    # Atributos realacionados con el suavizado del giro
    prev_angle_error = 0.0
    angle_error_acumulation = 0.0

    # -------- #
    # Atributos para corregir el "error" que se produce al pasar de la simulación al robot real

    # Constante para corregir el error de la detección del ángulo laser.
    lidar_angle_error = -100
    # Sentido horario (1 sí, -1 no)
    is_clockwise = -1

    # Atributos para corregir el error en la distancia
    error_min_distance = -0.15
    error_max_distance = -1.00

    min_distance = MIN_DISTANCE + error_min_distance
    max_distance = MAX_DISTANCE + error_max_distance

    # Centro con el error del LIDAR (el ángulo del laser).
    centre = CENTRE + lidar_angle_error
    min_front = MIN_FRONT + lidar_angle_error
    max_front = MAX_FRONT + lidar_angle_error

    def actualizar_centro(self, new_centre = lidar_angle_error):
        self.centre     = CENTRE + new_centre
        self.min_front  = MIN_FRONT + new_centre
        self.max_front  = MAX_FRONT + new_centre


    # -------- #

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.actualizar_centro()

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #

        # El conjunto de valores tal que: (indice de [0, ANGLE*2] , distancia)
        range_values = list(enumerate(ranges[self.min_front : self.max_front]))
        # Conjunto ordenado por la distancia (x es la tupla: x[0] indice, x[1] distanca)
        range_values.sort(key= lambda x: x[1])

        # Distancia menor y su índice.
        #  El índice es importante porque nos ayuda a saber el ángulo donde se encuentra la persona (p.ej.: a 20º)
        index, distance = range_values[0]

        print("self.min_distance: ", self.min_distance)
        print("distance: ", distance)

        # Avanza si está entre dos umbrales de distancia
        if self.min_distance < distance < self.max_distance:
            # Idea: a más distancia, más velocidad (pero con límites)
            vx = min(distance - self.min_distance, MAX_VEL)
            # Cuando está muy cerca va muy lento, corrección:
            vx = max(vx, MIN_VEL)

            # El índice estará entre 0 y 2·Ángulo  (o angulo_izq + angulo_der)
            #   Por lo tanto, desplazamos el indice para que sea el ángulo real.
            angle = index + self.min_front

            angle_error = self.is_clockwise * (self.centre - angle)
            print(f"angle_error: {angle_error}")

            # Pasamos la diferencia de ángulos a radianes
            wz = angle_error / pi

            self.prev_angle_error += angle_error
            self.angle_error_acumulation = 0.0 if angle_error == 0 else (self.angle_error_acumulation + angle_error)

            # Aplicamos las constantes para suavizar las velocidades líneales y angulares (respectivamente )
            vx *= VEL_SMOOTH_FACTOR
            wz = (wz * ANGLE_SMOOTH_FACTOR +
                  self.prev_angle_error * DERIVATE_SMOOTH_FACTOR +
                  self.angle_error_acumulation * INTEGRAL_SMOOTH_FACTOR)

            print(f"vx: {vx} |  wz: {wz}")
            print("--------")

        else:
            vx = 0.0
            wz = 0.0
            self.prev_angle_error = 0.0
            self.angle_error_acumulation = 0.0

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
