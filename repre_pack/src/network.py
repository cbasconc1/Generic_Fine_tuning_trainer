#!/usr/bin/python3

import os
import rospy
from std_msgs.msg import String
from repre_pack.msg import Instruction
from darknet_ros_msgs.msg import BoundingBoxes

import devices_beto_sp as pj
import levels as lvls


import tensorflow as tf
from transformers import TFAutoModel, AutoTokenizer, AutoModelForSequenceClassification
import pickle
import direcciones as dirs
import threading
import time
import datetime

class Network:
    def __init__(self):
        rospy.init_node('Neural_network', anonymous=False)
        # Subscriptores
        self.gesture_subscriber_1 = rospy.Subscriber(rospy.get_param("Neural_network/gesture_value_1_subs"), 
                                                      String, self.callback_gesture_1)
        #self.gesture_subscriber_2 = rospy.Subscriber(rospy.get_param("Neural_network/gesture_value_2_subs"), 
         #                                             String, self.callback_gesture_2)
        self.voice_subscriber = rospy.Subscriber(rospy.get_param("Neural_network/voice_value_subs"), 
                                                 String, self.callback_voice)
        
        self.yolo_subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", 
                                                BoundingBoxes, self.callback_yolo)
        
        # Publicadores
        self.command_publisher_dev = rospy.Publisher(rospy.get_param("Neural_network/final_device"), 
                                                     Instruction, queue_size=1)

        self.clasificador_devices, self.tokenizador_Devices = pj.model_build_devices()
        self.clasificador_levels, self.tokenizador_levels = lvls.model_build()

        self.pose1 = "none"
        self.location = "none"
        self.voices = None        
        self.device = None
        self.lvl = None

    def callback_gesture_1(self, string_msg):
        self.pose1 = string_msg.data

    def callback_voice(self, string_msg):
        voices_data = string_msg.data
        voices_data = voices_data.lower()
        if "hola" in voices_data and "no" not in voices_data:
            voices_data = voices_data.replace("hola ", "")
            #self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
            #print("Antes de modelos: "+self.current_time)
            self.device = pj.answer(voices_data, self.tokenizador_Devices, self.clasificador_devices)
            #self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
            #print("Discriminador dispositivos: "+self.current_time)
            self.lvl = lvls.answer(voices_data, self.tokenizador_levels, self.clasificador_levels)
            #self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
            #print("Discriminador de niveles: "+self.current_time)
            self.funcion() 

    def callback_yolo(self, BoundingBoxes_msg):
            for bounding_box in BoundingBoxes_msg.bounding_boxes:
                if bounding_box.Class == "person":
                    location = (bounding_box.xmin + bounding_box.xmax) / 2
                    #print("Persona detectada en (x):"+str(location))
                                                    #213
                    if location > 0  and location < 426:
                     #   print("\n derecha:")
                        self.location = "derecha"
                                                    ###426
                    if location > 426  and location < 854:
                      #  print("\n centro")
                        self.location = "centro"
                                                    ##639
                    if location >854  and location < 1280:
                       # print("\n izquierda")
                        self.location = "izquierda"

    def funcion(self):
        temp_dev = int(self.device)
        temp_lvl = int(self.lvl)
        if temp_dev is not 7 and temp_lvl is not 3 and temp_lvl is not None :
            if  temp_dev == 8:#cuando haya nivel pero no hay device
                self.device = dirs.obtener_salida(self.pose1, self.location)
                print("resultado dispo predicho : "+self.device)
                if self.device is None or self.device is "7":
                    print("información no suficiente")
                else:
                    self.command_publisher_dev.publish(str(self.device),str(temp_lvl))
            else:
                self.command_publisher_dev.publish(str(temp_dev),str(temp_lvl))


        self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
        print("Se va a cliente: "+self.current_time)        
        self.pose1 = "none"
        self.voices = None
        self.location = "none"
        self.lvl= None

    def simulate_voice_input(self):
        while True:
            user_input = input("INGRESA COMANDO Y LUEGO PRESIONA ENTER: ")
            #current_time = datetime.datetime.now().strftime("%S,%f")
            #print(current_time)
            msg = String()  # Crear un objeto String
            msg.data = user_input  # Asignar la cadena de caracteres a data
            self.callback_voice(msg)  # Llamar a callback_voice con el objeto String



if __name__ == "__main__":
    try:
        network = Network()
        print("Los modelos han sido importados correctamente.")
        threading.Thread(target=network.simulate_voice_input, daemon=True).start()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
