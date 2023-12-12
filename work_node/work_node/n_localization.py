#!/usr/bin/env python3

# Librerias de inicializacion de ros2
import rclpy
from rclpy.node import Node

# librerias de mensajes
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import LaserScan

# Librerias de navegacion
from nav_msgs.msg import Odometry
#import tf_transformations as tf

# Librerias utilitarias
import numpy as np
import math
import json


class local_objetivo(Node):
    ruta_almacenamiento = '/home/elio/ws_ros/archivos_v'
    def __init__(self):
        super().__init__('Nodo_localizacion')

        # Inicializar variables
        self._init_vars()

        # Inicializar subscriptores
        self._init_subs()

        # Iniciar publicador 
        self._init_pubs()

        # Iniciador de timer
        self.timer_proceso = self.create_timer(
            0.5,
            self.proceso_localizacion
        )
            
    def _init_vars(self):
        # Variables de validacion
        self.val_obj = False
        self.val_localizando = False
        self.val_localizada = False
        self.contador_p = 0

        # Variables de pose de robot
        self.robot_pose = None
        self.robot_angle = None
        self.scan_data = None
        self.laser_info: LaserScan = None

        # Personas
        self.lista_objetos = []
        self.ruta_archivo = 'Lista_personas.json'
        self.ruta_archivo_f = self.__class__.ruta_almacenamiento + '/' + self.ruta_archivo
        self.datos_json = self.cargar_datos(self.ruta_archivo_f)

    def _init_subs(self):
        # Creacion de subscriptor de imagen
        self.subs_obj_encontrado = self.create_subscription(
            Bool,
            '/PersonaEncontrada',
            self._callback_p,
            10
        )
        self.subs_obj_encontrado

        # Lectura de posible posicion de la persona
        self.subs_obj_posicionado = self.create_subscription(
            Point,
            '/D_objetivo',
            self._callback_pos,
            10
        )
        self.subs_obj_posicionado

        # Lectura de posicion de robot
        # Subscripcion para la lectura de la odometria
        self.pose_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.pose_callback, 
            10)
        self.pose_sub

        # Subscripcion para la lectura del sensor.
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10)
        self.scan_sub 


    def _init_pubs(self):
        # Creacion de publicador de persona_encontrada
        self.publicador_local = self.create_publisher(
            Bool,
            '/P_localizacion',
            10
        )
        # Publicacion de movimiento controlado para la localizacion
        self.publicador_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publicador de vuelta a casa
        self.publicador_casa = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

    def _callback_p(self, msg: Bool):
        self.val_obj = msg.data

        if self.val_obj:
            self.contador_p += 1
            self.get_logger().info("Persona encontrada... Localizando.")
        else:
            self.contador_p = 0
            self.get_logger().info("Buscando persona")

    # Seccion de callbacks de subscriptores
    # Lectura de sensor lidar
    def scan_callback(self, msg):
        # Solo se toma los datos procedentes de 50 grados al frente del robot
        ranges = msg.ranges

        rangos_deseados = []

        # Obtener los rangos en el rango deseado
        rangos_deseados.extend(ranges[-15:])
        rangos_deseados.extend(ranges[:15])

        self.scan_data = rangos_deseados
        self.laser_info = msg
        #print(rangos_deseados)

        #self.num_laser_points = len(scan_data.ranges)
    
    # Lectura de pose actual
    def pose_callback(self, pose_data):
        self.robot_pose = pose_data.pose.pose
        _, _, self.robot_angle = self.transfrom_from_quaternion_to_eular(self.robot_pose.orientation)

    def _callback_pos(self, msg: Point):
        #orden de moivimiento
        orden_movimiento = Twist()

        # Lectura de posicion de imagen
        dif_x = msg.x
        tam_p = msg.z
        # Envio de posicion
        val_giro = 0.1
        # Validacion de magnitud
        if dif_x < 0:
            val_giro = -val_giro

        #print(dif_x)
        if self.val_localizando and not self.val_localizada:
            orden_movimiento.linear.x = 0.0
            orden_movimiento.angular.z = 0.0
            
            if abs(dif_x) > 50:
                orden_movimiento.linear.x = 0.0
                orden_movimiento.angular.z = val_giro
            elif tam_p < 65000:
                orden_movimiento.linear.x = 0.2
                orden_movimiento.angular.z = 0.0
                
            
            if abs(dif_x) < 40 and tam_p > 65000:
                robot_position = [self.robot_pose.position.x, self.robot_pose.position.y]
                robot_orientation = self.robot_angle

                robot_orientation = self.robot_angle

                val_r = self.scan_data[15]

                local_x = float(val_r * np.cos(robot_orientation) + robot_position[0])
                local_y = float(val_r * np.sin(robot_orientation) + robot_position[1])

                vector_personas = np.array([local_x, local_y])

                self.update_persona_points(vector_personas, 1)

                self.guardar_datos(self.ruta_archivo_f, self.datos_json)

                self.val_localizada = True

                self.enviar_casa()

                self.__node.destroy_node()
                self.__node.shutdown()

            self.publicador_vel.publish(orden_movimiento)


    def proceso_localizacion(self):
        msg = Bool()
        self.val_localizando = False
        if self.contador_p >= 1 and not self.val_localizada:
            self.val_localizando = True

        msg.data = self.val_localizando

        self.publicador_local.publish(msg)

    def enviar_casa(self):
        posicion_0 = PoseStamped()

        posicion_0.header.frame_id = 'map'
        posicion_0.header.stamp.sec = 0
        posicion_0.pose.position.x = 0.0
        posicion_0.pose.position.y = 8.0
        posicion_0.pose.position.z = 0.0
        posicion_0.pose.orientation.x = 0.0
        posicion_0.pose.orientation.y = 0.0
        posicion_0.pose.orientation.z = 0.0
        posicion_0.pose.orientation.w = 0.0

        self.publicador_casa.publish(posicion_0)

    #SECCION DE FUNCION UTILITARIAS
    def cargar_datos(self, ruta_archivo):
        datos = []
        try:
            with open(ruta_archivo, 'r') as filename:
                file_content = filename.readlines()

                for line in file_content:
                    datos.append(str(line))
        except FileNotFoundError:
            datos = {}  # Si el archivo no existe, crea un diccionario vacío
        return datos
    
    # Función para guardar datos en un archivo JSON
    def guardar_datos(self, ruta_archivo, datos):
        with open(ruta_archivo, 'w') as archivo:
            json.dump(datos, archivo, indent=4)

     # actualizacion de personas en la lista
    def update_persona_points(self, point, threshold = 0.8):
        if len(self.lista_objetos) <= 0:
            self.lista_objetos.append(point)

            n_objeto = f"Persona_{len(self.lista_objetos)}"

            self.datos_json[n_objeto] = {'Coordenada': list(point)}
        # #if new_points != None:
        # elif not any(np.linalg.norm(point - existing_point) < threshold for existing_point in self.lista_objetos):
        #     self.lista_objetos.append(point)

        #     n_objeto = f"Persona_{len(self.lista_objetos)}"

        #     self.datos_json[n_objeto] = {'Coordenada': list(point)}
    
    def transfrom_from_quaternion_to_eular(self, q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args = None):
    rclpy.init(args = args)

    # Inicializacion de objetos
    objeto_local = local_objetivo()
    rclpy.spin(objeto_local)

    # Finalizacion de nodo
    objeto_local.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    