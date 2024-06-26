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

# Constante para inidcar la "base" en la que se va a trabajar 
#    (indiferentemente del número de muestras que haga el LIDAR)
ORIGINAL_MAX_RANGE = 360

# Constante para indicar el rango de muestras que se tienen en cuenta a uno de los lados 
#    del centro (la posición en la que debería estar la persona), es decir, si la constante 
#    vale 20 y el "centro" es 180, tendrá en cuenta desde 160 (centro - 20) hasta 200 (centro + 20).
ANGLE_IF_ORIGINAL_MAX_RANGE = 20

# Constante para indicar la distancia mínima a la que puede estar el robot (en metros)
MIN_DISTANCE = 0.45
# Constante para indicar la distancia máxima de detección 
MAX_DISTANCE = 2.00
# Constante para indicar la velocidad máxima (en m/s) a la que puede ir (teóricamente 
#    y sin tener en cuenta los factores de "suavizado")
MAX_VEL = 6.67
# Constante para indicar la velocidad mínima (en m/s) a la que debe ir 
#    (sin tener en cuenta los factores de "suavizado")
MIN_VEL = 0.20

# Constantes para suavizar la velocidad de movimiento tanto lineal como angular 
#    (1ª y 2ª constante respectivamente)
VEL_SMOOTH_FACTOR = 0.50
ANGLE_SMOOTH_FACTOR = 0.10

# Constante para indicar el tanto por 1 del ángulo que detectará si va a la velocidad máxima, 
#    es decir, si vale 0.4 indicaría que, si está a la velocidad máxima, el rango de detección 
#    será solo de un 40 % del valor base. Por ejemplo, si es el ángulo es de 20º, se tendrá 
#    en cuenta solo el rango de 8º (que es el 40 % de 20º) desde el centro.
MIN_AJUSTE_ANGULO = 0.4

# -- FIN CONSTANTES -- #

# -- FUNCIONES DE APOYO -- #
# Esto es para que si "valor" vale "min_v" tengamos 1, si "valor" vale "max_v", valga "valor_minimo", y
#  entre medias, que tenga un valor proporcional a la distancia entre entre 1 y "valor_minimo"
def normalizar_a_la_inversa(valor, min_v=MIN_VEL, max_v=MAX_VEL, valor_minimo=MIN_AJUSTE_ANGULO):
    return 1 - (1 - valor_minimo) * (valor - min_v) / (max_v - min_v)

# Función para redondear bien la unidad porque, debido a un fallo en la versión de python instalada (y 
#    parece que del tratamiento de los flotantes de Python3 en general), la función "round" redondea 0.5 
#    como 0 en vez de como 1, en contra de la definición del redondeo (pero con el resto de decimales 
#    va bien, por ejemplo "round(0.05, 1)" sí que lo redonda a 0.1).
def redondear_bien_a_la_unidad(entrada):
    entrada_sin_decimales = int(entrada)
    valor = entrada - entrada_sin_decimales
    if valor < 0.5:
        return entrada_sin_decimales
    else:
        return entrada_sin_decimales + 1
# -- FIN FUNCIONES DE APOYO -- #

class PersonFollower(Node):

    # --- Atributos "base" --- #
    # Centro de la parte frontal del robot.
    centre = 0
    # Esta variable está para simplificarnos el código relacionado con el giro en casos especiales.
    middle_angle = ORIGINAL_MAX_RANGE // 2
    # Valor mínimo del rango en el que se detecta respecto del centro
    max_angle = ORIGINAL_MAX_RANGE
    # Valor máximo del rango en el que se detecta respecto del centro   
    velocidad_anterior = MIN_VEL
    # El número de mediciones que recibe
    max_angle = ORIGINAL_MAX_RANGE
    # El rango de mediciones a tener en cuenta
    min_front = centre - ANGLE_IF_ORIGINAL_MAX_RANGE
    max_front = centre + ANGLE_IF_ORIGINAL_MAX_RANGE
    # -------- #

    # -------- #
    # Atributos para corregir el "error" que se produce al pasar de la simulación al robot real

    # Constante para corregir el error de la detección del ángulo laser.
    lidar_angle_error = -542
    # Sentido horario de la detección (1 sí, -1 no)
    is_clockwise = 1

    # Atributos para corregir el error en la distancia en caso de que el sensor detecte más o menos
    error_min_distance = 0.0
    error_max_distance = 0.0
    
    min_distance = MIN_DISTANCE + error_min_distance
    max_distance = MAX_DISTANCE + error_max_distance
    # -------- #

    # Método para obtener una lista de distancias (e índices) ordenada 
    #   La salida es una lista de tuplas (ángulo, distancia)
    def lista_distancias_ordenada(self, ranges):

        # El list-enumerate es para tener una lista cuyos valores sea tal que: (índice , distancia)
        #   (Nota: índice === ángulo desde el que se ha detectado la distancia)
        ranges = list(enumerate(ranges))

        # Caso en el que hay un cambio de signo en medio.
        if self.min_front < 0 and self.max_front >= 0:
            # valores a la izquierda del 0
            valores_izq = ranges[self.min_front:]
            # valores desde el 0 hasta el máximo de la derecha (el +1 es para pillar también el último valor)
            valores_der = ranges[: self.max_front + 1]
        else:
            # valores a la izquierda del "centro"
            valores_izq = ranges[self.min_front : self.centre]
            # valores desde el centro hasta el máximo de la derecha (el +1 es para pillar también el último valor)
            valores_der = ranges[self.centre : self.max_front + 1]

        # Nota: Aquí los valores aún están desordenados aunque la variable indique lo contrario.
        distancias_ordenadas = valores_izq + valores_der

        # Ordenamos el conjunto por la distancia (x es la tupla: x[0] índice, x[1] distancia)
        distancias_ordenadas.sort(key= lambda x: x[1])

        return distancias_ordenadas
    # -------- #

    def asignar_constantes_y_correccion_angulo(self, num_mediciones, vel_anterior = velocidad_anterior,
                                               correccion_angulo_robot = lidar_angle_error):

        # Calculamos el ángulo "ANGLE_IF_ORIGINAL_MAX_RANGE" en base al número de muestras que se leen ahora
        angle = num_mediciones * ANGLE_IF_ORIGINAL_MAX_RANGE // ORIGINAL_MAX_RANGE

        # Ajustamos el ángulo de detección inversamente proporcional: 1 es la distancia mínima, "valor_minimo" es
        #   la distancia máxima y el ángulo de detección varia en función de la velocidad anterior
        angle = redondear_bien_a_la_unidad((angle * normalizar_a_la_inversa(vel_anterior)))

        # -- Asignamos las variables:
        # Se asume que el centro está a la mitad del conjunto de mediciones
        self.centre = num_mediciones // 2
        # Esta variable está para simplificarnos el cálculo del giro más adelante.
        self.middle_angle = num_mediciones        
        # Valor mínimo del rango en el que se detecta respecto del centro
        self.min_front = self.centre - angle
        # Valor máximo del rango en el que se detecta respecto del centro                                           
        self.max_front = self.centre + angle
        # El número de mediciones
        self.max_angle = num_mediciones

        # Ajustamos las variables a las correciones para el robot.
        self.centre += correccion_angulo_robot
        self.middle_angle += correccion_angulo_robot
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

        self.asignar_constantes_y_correccion_angulo(num_mediciones=len(ranges), vel_anterior=self.velocidad_anterior)
        # Obtenemos una lista ordenada de menor distancia a mayor de los ángulos y las distancias que queremos detectar
        range_values = self.lista_distancias_ordenada(ranges)

        # Para que no dé error ni se bloquee si no detecta valores en el rango a "observar"
        if len(range_values) == 0:
            return None

        #  "angle" nos sirve para saber el ángulo donde se encuentra la persona (p.ej.: si es 20, a 20º)
        angle, distance = range_values[0]
        print("-> self.min_distance: ", self.min_distance)
        print("-> self.max_distance: ", self.max_distance)
        print("-> distance: ", distance)
        print("-> angle: ", angle)

        # Se mueve si está entre dos umbrales de distancia
        if self.min_distance < distance < self.max_distance:
            # A más distancia, más velocidad (pero con límites)
            vx = min(distance - self.min_distance, MAX_VEL)
            # Pero cuando está muy cerca va muy lento; nuestra corrección:
            vx = max(vx, MIN_VEL)

            # Aquí se calcula la diferencia entre el ángulo detectado y el centro, pero, si el centro no 
            #    está a la mitad de las mediciones (por ejemplo, porque es 0º, en vez de 180º), todos los 
            #    ángulos de giro podrían ser de un solo signo y el sistmea necesita que sean de 2 signos: 
            #    positivo para girar en un sentido y negativo para girar en el sentido opuesto. 
            #
            #    Nota: Si tuviesemos de que el rango de detección nunca va a traspasar los de extremos (0 y 
            #    self.max_angle), sería solo "angle_error = (self.centre - angle)"
            angle_error = (self.max_angle - angle) if (angle >= self.middle_angle) else (self.centre - angle)

            # Cambia el signo dependiendo del sentido de las detecciones del LIDAR
            angle_error *= self.is_clockwise
            print(f"angle_error: {angle_error}")

            # Se adapta el ángulo para que esté entre 0 y 360 (es decir, "ORIGINAL_MAX_RANGE") para así 
            #     poder pasarlo a radianes (es necesario para la velocidad angular).
            angle_error = angle_error * ORIGINAL_MAX_RANGE / self.max_angle
            print("angle_error en base 360 ", angle_error)

            # Si es múltiplo de 360 (es decir, ORIGINAL_MAX_RANGE), tiene que ser 0
            angle_error = angle_error if (angle_error % ORIGINAL_MAX_RANGE) != 0 else 0
            print("angle_error post-transformaciones", angle_error)

            # Pasamos la diferencia de ángulos a radianes
            wz = angle_error / pi

            # Aplicamos las constantes para suavizar las velocidades líneales y angulares (respectivamente)
            vx *= VEL_SMOOTH_FACTOR
            wz *= ANGLE_SMOOTH_FACTOR

            # La velocidad anterior tiene que ser, como mínimo, el mínimo de la velocidad para simplificar los
            #    cálculos de la función de normalización
            self.velocidad_anterior = max(vx, MIN_VEL)
            print(f"vx: {vx} |  wz: {wz}")
            print("--------")

        else:
            # Si no detecta a nadie, se queda quieto
            vx = 0.0
            wz = 0.0
            # El siguietne valor es "MIN_VEL" en vez de 0 para los cálculos de la función de normalización
            self.velocidad_anterior = MIN_VEL

        # Se mandan las velocidades al robot
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
