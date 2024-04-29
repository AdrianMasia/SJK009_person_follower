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

# En el caso de 360
ANGLE_IF_ORIGINAL_MAX_RANGE = 30
ORIGINAL_MAX_RANGE = 360

MIN_DISTANCE = 0.50
MAX_DISTANCE = 2.50
MAX_VEL = 6.67
MIN_VEL = 0.20

VEL_SMOOTH_FACTOR = 0.50 # 0.5
ANGLE_SMOOTH_FACTOR = 0.15 # 0.15

# Esto molesta y supongo que si se ajustan los valores aportaría poco.
# DERIVATE_SMOOTH_FACTOR = 0.00010 # 0.010
# INTEGRAL_SMOOTH_FACTOR = 0.0000010 # 0.00010

class PersonFollower(Node):

    # ----- #
    centre = 0
    max_angle = 360

    # Atributos realacionados con el suavizado del giro
    prev_angle_error = 0.0
    angle_error_acumulation = 0.0

    # -------- #
    # Atributos para corregir el "error" que se produce al pasar de la simulación al robot real

    # Constante para corregir el error de la detección del ángulo laser.
    lidar_angle_error = -540
    # Sentido horario (1 sí, -1 no)
    is_clockwise = -1

    # Atributos para corregir el error en la distancia
    error_min_distance = -0.15
    error_max_distance = -1.00

    min_distance = MIN_DISTANCE + error_min_distance
    max_distance = MAX_DISTANCE + error_max_distance

    def lista_distancias_ordenada(self, ranges):

        # El list-enumerate es para tener un alista cuyos valores sea tal que: (indice , distancia)
        ranges = list(enumerate(ranges))

        # Caso en el que hay un cambio de signo en medio.
        if self.min_front < 0 and self.max_front >= 0:
            valores_izq = ranges[self.min_front:]
            valores_der = ranges[: self.max_front + 1]
        else:
            valores_izq = ranges[self.min_front : self.centre]
            valores_der = ranges[self.centre : self.max_front + 1]

        distancias_ordenadas = valores_izq + valores_der

        # Conjunto ordenado por la distancia (x es la tupla: x[0] indice, x[1] distanca)
        distancias_ordenadas.sort(key= lambda x: x[1])

        return distancias_ordenadas

    def asignar_constantes_y_correccion_angulo(self, num_mediciones, correccion_angulo_robot = lidar_angle_error):

        #angle = int( num_mediciones * ANGLE_IF_ORIGINAL_MAX_RANGE / ORIGINAL_MAX_RANGE)
        angle = num_mediciones * ANGLE_IF_ORIGINAL_MAX_RANGE // ORIGINAL_MAX_RANGE

        # asignamos las variables
        self.centre = num_mediciones // 2
        self.min_front = self.centre - angle
        self.max_front = self.centre + angle
        self.max_angle = num_mediciones

        # ajustamos el centro a los errores
        self.centre += correccion_angulo_robot
        self.min_front += correccion_angulo_robot
        self.max_front += correccion_angulo_robot
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

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges
        #
        # your code for computing vx, wz
        #

        self.asignar_constantes_y_correccion_angulo(len(ranges))
        range_values = self.lista_distancias_ordenada(ranges)

        # Distancia menor y su índice.

        # Forma "eficaz" de que no de error si no detecta ángulo
        if len(range_values) == 0:
            return None

        #  "angle" nos sirve para saber el ángulo donde se encuentra la persona (p.ej.: a 20º)
        angle, distance = range_values[0]
        print("self.min_distance: ", self.min_distance)

        print("distance: ", distance)

        # Avanza si está entre dos umbrales de distancia
        if self.min_distance < distance < self.max_distance:
            # Idea: a más distancia, más velocidad (pero con límites)
            vx = min(distance - self.min_distance, MAX_VEL)
            # Cuando está muy cerca va muy lento, corrección:
            vx = max(vx, MIN_VEL)

            angle_error = self.is_clockwise * (self.centre - angle)
            print(f"angle_error: {angle_error}")

            # Normalizamos el ángulo para que esté entre 0 y 360
            angle_error = angle_error * ORIGINAL_MAX_RANGE / self.max_angle

            # Pasamos la diferencia de ángulos a radianes
            wz = angle_error / pi

            self.prev_angle_error += angle_error
            self.angle_error_acumulation = 0.0 if angle_error == 0 else (self.angle_error_acumulation + angle_error)

            # Aplicamos las constantes para suavizar las velocidades líneales y angulares (respectivamente )
            vx *= VEL_SMOOTH_FACTOR
            #wz = (wz * ANGLE_SMOOTH_FACTOR + self.prev_angle_error * DERIVATE_SMOOTH_FACTOR + self.angle_error_acumulation * INTEGRAL_SMOOTH_FACTOR)
            wz = wz * ANGLE_SMOOTH_FACTOR

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
