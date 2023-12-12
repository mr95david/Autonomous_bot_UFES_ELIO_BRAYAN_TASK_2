#!/usr/bin/env python3

# Librerias de inicializacion de ros2
import rclpy
from rclpy.node import Node

# Librerias utilitarios
import time
import os
import numpy as np

# Librerias de deteccion de imagen
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

#import herramientas de deteccion
from ultralytics import YOLO

class deteccion_2(Node):
    ruta_archivos = '/home/elio/nombres'
    def __init__(self):
        # Inicializacion clase super
        super().__init__('Nodo_deteccion')

        # Inicializacion variables 
        self._init_vars()

        #Inicializacion de subscriptores
        self._init_subs()

        # Inicializacion de publicadores
        self._init_pubs()

        # Creacion de timers
        self.timer_temp = self.create_timer(
            0.5,
            self.persona_encontrada
        )

        # Validacion de inicio de nodo
        self.get_logger().info("Nodo iniciado correctamente")

    def _init_vars(self):
        # Variables de adquisicion de imagen
        self.val_from_img = Image()
        self.lista_nombres = self.cargar_datos()

        # Valores de estudio de imagenes
        self.cof = 0.5
        self.font_scale = 1
        self.thickness = 1
        self.model = YOLO("yolov8n.pt")
        self.set_colors = np.random.randint(
            0,
            255,
            size = (len(self.lista_nombres), 3),
            dtype = "uint8"
        )

        # Variables de validacion
        self.val_persona: bool = False
        self.tamanho_p: int = 0
        self.pos_x_g: int = None
        self.pos_x_p: int = None

    def _init_subs(self):
        # Creacion de subscriptor de imagen
        self.subs_img_data = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_img,
            10
        )
        self.subs_img_data

    def _init_pubs(self):
        # Creacion de publicador de persona_encontrada
        self.publicador_persona = self.create_publisher(
            Bool,
            '/PersonaEncontrada',
            10
        )
        # Crear interfaz de relacion de posicion
        self.publicador_pos = self.create_publisher(
            Point,
            '/D_objetivo',
            10
        )

    def listener_img(self, msg):
        # Validaciones de mensajes
        msg1 = Point()
        self.val_object = False
        self.tamanho_p = 0.0
        self.pos_x_g = None
        self.pos_x_p = None
        
        # Recibe el valor proporcionado por la camara
        self.val_from_img = msg

        # Convertimos el valor a un vector legible por cv2
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(
            self.val_from_img,
            desired_encoding = "bgr8"
        )

        # Conversion de tamaño de imagen
        porcentaje_escala = 60
        width = int(cv_img.shape[1] * porcentaje_escala / 100)
        height = int(cv_img.shape[0] * porcentaje_escala / 100)
        dim = (width, height)
        
        # resize image
        resized = cv2.resize(cv_img, dim, interpolation = cv2.INTER_AREA)

        # Dibujar punto medio de imagen
        resized = self.dibujar_centro(resized, height, width)

        # People detection
        resized = self.deteccion_objetos(resized)
        
        # Publicacion de mensaje general
        if self.pos_x_p != None:
            msg1.x = float(width//2 - self.pos_x_p)
        else:
            msg1.x = 0.0
        msg1.z = float(self.tamanho_p)
        self.publicador_pos.publish(msg1)

        # Visualizacion de img
        cv2.imshow("Visualizacion camara", resized)
        cv2.waitKey(1)

    def deteccion_objetos(self, img):
        # Deteccion de prediccion de persona
        results = self.model.predict(img, conf = self.cof)[0]
        #print(results)

        for data in results.boxes.data.tolist():
            
            # get the bounding box coordinates, confidence, and class id 
            xmin, ymin, xmax, ymax, confidence, class_id = data
            # converting the coordinates and the class id to integers
            xmin = int(xmin)
            ymin = int(ymin)
            xmax = int(xmax)
            ymax = int(ymax)
            class_id = int(class_id)

            color = [int(c) for c in self.set_colors[class_id]]
            
            text = f"{self.lista_nombres[class_id]}: {confidence:.2f}"

            texto, evaluacion = text.split(":")
            texto = texto.strip()
            evaluacion = evaluacion.strip()
            #print(texto, evaluacion)

            (text_width, text_height) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=self.font_scale, thickness=self.thickness)[0]
            
            text_offset_x = xmin
            text_offset_y = ymin - 5
            
            box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))

            if texto.startswith("person") and float(evaluacion)*100 > 82.0 and ((ymax-ymin) * (xmax-xmin)) >= 45000 :
                #print((ymax-ymin) * (xmax-xmin))
                self.val_object = True
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color=color, thickness= self.thickness)

                overlay = img.copy()

                cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
                # add opacity (transparency to the box)
                img = cv2.addWeighted(overlay, 0.6, img, 0.4, 0)
                # now put the text (label: confidence %)

                img = self.dibujar_centro(img, ymax-ymin, xmax-xmin, (0, 0, 255), xinit = xmin, yinit = ymin)

                # Calcular tamano de reconocimiento
                self.tamanho_p = (ymax-ymin) * (xmax-xmin)
                self.pos_x_p = (xmax-xmin)//2 + xmin

                cv2.putText(img, text, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=self.font_scale, color=(0, 0, 0), thickness=self.thickness)

        #Creacion de boundings
        return img
    
    #SECCION DE FUNCION UTILITARIAS
    def cargar_datos(self):
        datos = []
        try:
            with open(f'{self.__class__.ruta_archivos}/object_names.txt', 'r') as filename:
                file_content = filename.readlines()

                for line in file_content:
                    datos.append(str(line))
        except FileNotFoundError:
            datos = []  # Si el archivo no existe, crea un diccionario vacío
        return datos
    
    # Funcion para dibujar el centro en una imagen, 
    def dibujar_centro(self, img, img_h, img_w, color = (0, 255, 0), xinit = 0, yinit = 0):

        mid_y = img_h//2 + yinit
        mid_x = img_w//2 + xinit
        
        # color en formato BGR (verde en este caso)
        thickness = -1  # espesor -1 dibuja un punto lleno
        cv2.circle(img, (mid_x, mid_y), 5, color, thickness)

        return img
    
    # Callback de persona encontrada
    def persona_encontrada(self):
        msg = Bool()
        msg.data = self.val_object
        self.publicador_persona.publish(msg)

def main(args = None):
    rclpy.init(args = args)

    # Inicializacion de objetos
    objeto_visualizacion = deteccion_2()
    rclpy.spin(objeto_visualizacion)

    # Finalizacion de nodo
    objeto_visualizacion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
