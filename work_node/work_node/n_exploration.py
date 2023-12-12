#!/usr/bin/env python3

import rclpy
import numpy as np
import time
import random
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped
import tf_transformations
from nav_msgs.msg import OccupancyGrid, Odometry

# Libreria temporal
#from rclpy.action import ActionClient

class nav_origin(Node):
    def __init__(self):
        super().__init__('Nodo_navegacion_origen')
        self.val_persona = False
        self.val_mapa = False
        self.pos_objetivo = None
        self.unexplored_cells = []
        self.first_unknown = None
        self.objetive = 0
        self.time_inicio = None
        self.time_dis = time.time()
        self.poserobot: Pose = Pose()
        self.t_vel = Twist()
        self.pose: PoseStamped = PoseStamped()
        self.linear_velocity = 0
        self.angular_velocity = 0
        self._init_subs()
        self._init_pub()


        self.timer = self.create_timer(
            1,
            self.loop
        )

    def _init_subs(self):

        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self._cmd_vel_callback,
            10
        )
        self.sub_val_find = self.create_subscription(
            Bool,
            '/PersonaEncontrada',
            self._listener_val,
            1
        )
        self.sub_robotodom_find = self.create_subscription(
            Odometry,
            '/odom',
            self._listener_odom,
            10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback_2,
            1
        )
        
    def _cmd_vel_callback(self, msg: Twist):
        # Acceder a los datos de velocidad lineal y angular
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z


    def _init_pub(self):
        self.publicador_obj = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.publicador_vel = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
            10
        )

    def _listener_val(self, msg: Bool):
        self.val_persona = bool(msg.data)
        print(bool(msg.data))
        #print(punto_aletorio)
        # if self.val_persona:

        #     # Seccion de detencion de nodo
        #     self.pose.header.frame_id = 'map'
        #     self.pose.header.stamp.sec = 0
        #     self.pose.pose.position.x = self.poserobot.position.x
        #     self.pose.pose.position.y = self.poserobot.position.y
        #     self.pose.pose.position.z = self.poserobot.position.z
        #     self.pose.pose.orientation.x = self.poserobot.orentation.x
        #     self.pose.pose.orientation.y = self.poserobot.orientation.y
        #     self.pose.pose.orientation.z = self.poserobot.orientation.z
        #     self.pose.pose.orientation.w = self.poserobot.orientation.w

        #     self.publicador_obj.publish(self.pose)
            
        #     self.publicador_obj.destroy()
        #     self.__node.destroy_node()
        #     self.__node.shutdown()
        

    def _listener_odom(self, msg: Odometry):
        self.poserobot.position.x = msg.pose.pose.position.x
        self.poserobot.position.y = msg.pose.pose.position.y
        
        self.poserobot.orientation.x = msg.pose.pose.orientation.x
        self.poserobot.orientation.y = msg.pose.pose.orientation.y
        self.poserobot.orientation.z = msg.pose.pose.orientation.z
        self.poserobot.orientation.w = msg.pose.pose.orientation.w

    def _map_callback_2(self, msg: OccupancyGrid):
        if not self.val_mapa:
            map_data = np.array(msg.data)
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            pos_ini_m_x = msg.info.origin.position.x
            pos_ini_m_y = msg.info.origin.position.y
            # print("punto incial", [pos_ini_m_x, pos_ini_m_y])

            matriz = map_data.reshape(width, height)

            nueva_matr = []
            cont = 0
            if (time.time()-self.time_dis) > 15:
                for fila in range(len(matriz)):
                    for col in range(len(matriz[fila])):
                        # if matriz[fila][col] != -1 and matriz[fila][col] !=0:
                        #     print('dato',matriz[fila][col])
                        if matriz[fila][col] == -1:
                            fila_n = fila * resolution + pos_ini_m_x
                            col_n = col * resolution + pos_ini_m_y
                            if [fila_n, col_n] not in self.unexplored_cells:
                                nueva_matr.append([fila_n, col_n])
                                self.time_dis = time.time()
            else:
                for fila in range(len(matriz)):
                    for col in range(len(matriz[fila])):
                        if matriz[fila][col] == -1:
                            fila_n = fila * resolution + pos_ini_m_x
                            col_n = col * resolution + pos_ini_m_y
                            if self.calculo_diferencia(fila_n, col_n) < 2.5 and [fila_n, col_n] not in self.unexplored_cells:
                                nueva_matr.append([fila_n, col_n])                 
            self.unexplored_cells = nueva_matr
            # print("ultimo", nueva_matr[-1])

    def loop(self):
        
        if self.time_inicio == None:
            self.time_inicio = time.time()
        
        val_def = self.calculo_diferencia(self.pose.pose.position.x, self.pose.pose.position.y)
        # Generar un número aleatorio para elegir la estrategia
        random_strategy = random.randint(1, 8)  # Genera un número aleatorio entre 1 y 5
        # Selección de estrategia basada en el número aleatorio generado
        if self.val_persona:
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp.sec = 0
            self.pose.pose.position.x = self.poserobot.position.x
            self.pose.pose.position.y = self.poserobot.position.y
            self.pose.pose.position.z = self.poserobot.position.z
            self.pose.pose.orientation.x = self.poserobot.orientation.x
            self.pose.pose.orientation.y = self.poserobot.orientation.y
            self.pose.pose.orientation.z = self.poserobot.orientation.z
            self.pose.pose.orientation.w = self.poserobot.orientation.w

            self.publicador_obj.publish(self.pose)
            
            self.publicador_obj.destroy()
            self.__node.destroy_node()
            self.__node.shutdown()
                    
        elif len(self.unexplored_cells) > 2 and (val_def < 1 or (time.time()-self.time_inicio) > 80 or (self.linear_velocity < 0.01)):
            if random_strategy == 1:
                pos_aleatorio = 0  # Estrategia 1: Seleccionar el primer elemento de la lista
            elif random_strategy == 2:
                pos_aleatorio = len(self.unexplored_cells) - 1  # Estrategia 2: Seleccionar el último elemento de la lista
            elif random_strategy == 3:
                pos_aleatorio = len(self.unexplored_cells) // 2  # Estrategia 3: Seleccionar la mitad de la lista
            elif random_strategy == 4:
                pos_aleatorio = (len(self.unexplored_cells) // 4) + (len(self.unexplored_cells) // 2)  # Estrategia 4: Seleccionar la mitad de la mitad de la lista
            else: 
                pos_aleatorio = np.random.randint(0, len(self.unexplored_cells)-1)
            
            punto_aletorio = self.unexplored_cells[pos_aleatorio]
            
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            
            
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp.sec = 0
            self.pose.pose.position.x = punto_aletorio[0]
            self.pose.pose.position.y = punto_aletorio[1]
            self.pose.pose.position.z = 0.0
            self.pose.pose.orientation.x = q_x
            self.pose.pose.orientation.y = q_y
            self.pose.pose.orientation.z = q_z
            self.pose.pose.orientation.w = q_w

            self.publicador_obj.publish(self.pose)
            # self.get_logger().info('Published goal pose')

            # Eliminar la celda explorada para avanzar a la siguiente en la siguiente iteración
            self.unexplored_cells.pop(0)  # Eliminar la primera celda explorada de la lista
            self.first_unknown = self.unexplored_cells[0] if self.unexplored_cells else None  # Actualizar la próxima celda objetivo
            self.time_inicio = None

    def calculo_diferencia(self, val_pos_x, val_pos_y):
        val_x = abs(self.poserobot.position.x - val_pos_x)
        val_y = abs(self.poserobot.position.y - val_pos_y)

        distancia = np.sqrt(val_x*2 + val_y*2)
        return distancia

def main(args = None):
    rclpy.init(args = args)
    objeto_devolver = nav_origin()
    rclpy.spin(objeto_devolver)

    objeto_devolver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()