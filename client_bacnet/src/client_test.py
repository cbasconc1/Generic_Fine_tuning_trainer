#!/usr/bin/python3

import time
import BAC0
import rospy
from repre_pack.msg import Instruction
from colorama import Fore, Style
import datetime, time
devices_name = ['luz_der', 'luz_izq', 'luz_cen','ventana_izq', 'ventana_cen', 'ventana_der', 'puerta','random'
                ,'nada'] 
lvl_sr_1 = [60,100,175]
lvl_sr_2 = [175,100,75]
class client_node:
    def __init__(self):
        self.dev, self.client = self.init_comunication()
        rospy.init_node('Client_node', anonymous=False)
        rospy.Subscriber(rospy.get_param("client_node/DEV_param"), Instruction, self.callback_1)
        self.rate = rospy.Rate(1000)

    def init_comunication(self):
        try:
            client = BAC0.lite("192.168.0.107/24")
            dev = BAC0.device("192.168.0.109", 123, client, poll=0)
            #dev["ventana_der1"] =  60
            #dev["puerta"] =  60
            print("Conectado al servidor")
            return dev, client
        except Exception as e:
            print(Fore.RED + f"Error: {e}" + Style.RESET_ALL)
            return None, None

    def callback_1(self, dev_msg): 
        try:
            #current_time = datetime.datetime.now().strftime("%M:%S,%f")
            #print(current_time)
            device_index = int(dev_msg.Device_Name)
            self.tmp_dev_callback_str = devices_name[device_index]
            self.tmp_lvl_callback = int(dev_msg.Level)    
            self.write()
        except ValueError:
             print(Fore.RED + "Invalid device index received." + Style.RESET_ALL)

    def write(self):
        print(Fore.GREEN + "Transmitiendo instrucción ...\n")
        print(str(self.tmp_lvl_callback)+"  "+str(self.tmp_dev_callback_str))
        #self.dev["ventana_der1"] =  60
        print("no se epudo")
        try:
            if self.tmp_dev_callback_str == "ventana_der":
                self.dev["ventana_der1"] = 100
                self.dev["ventana_der1"] = lvl_sr_2[self.tmp_lvl_callback]
                self.dev["ventana_der2"] = lvl_sr_2[self.tmp_lvl_callback]
            elif self.tmp_dev_callback_str == "ventana_cen":
                self.dev["ventana_cen1"] = lvl_sr_2[self.tmp_lvl_callback]
                self.dev["ventana_cen2"] = lvl_sr_1[self.tmp_lvl_callback]
            elif self.tmp_dev_callback_str == "ventana_izq":
                self.dev["ventana_izq1"] = lvl_sr_2[self.tmp_lvl_callback]
                self.dev["ventana_izq2"] = lvl_sr_1[self.tmp_lvl_callback]
            elif self.tmp_dev_callback_str == "puerta":
                self.dev["puerta"] = lvl_sr_2[self.tmp_lvl_callback]
            else: 
                self.dev[self.tmp_dev_callback_str] = self.tmp_lvl_callback * 50
            print("El dispositivo modificado fue: " + self.tmp_dev_callback_str)
            print("El nivel modificado fue: " + str(self.tmp_lvl_callback) + Style.RESET_ALL)   
        except Exception as e:
            print(Fore.RED + f"An exception occurred: {str(e)}" + Style.RESET_ALL)
            print(Fore.RED + "Transmisión no posible ...\n" + Style.RESET_ALL)

    def prueba(self):
        while not rospy.is_shutdown():
            if self.dispositivo_index <= 2:
                for valor1 in range(0, 101, 2):  # De 0 a 100 con paso 2
                    for decima in range(10):
                        valor_decimal = valor1 + (decima / 10.0)  # Combinar valor entero con décimas
                        self.write()
                        # time.sleep(0.1)  # Pausa de 0.1 segundos entre cambios de valor
                for valor2 in range(100, -1, -2):  # De 100 a 0 con paso -2
                    for decima in range(10):
                        valor_decimal = valor2 + (decima / 10.0)  # Combinar valor entero con décimas
                        self.write()
                        # time.sleep(0.2)  # Pausa de 0.valor22 segundos entre cambios de valor
            elif 2 < self.dispositivo_index <= 10:
                for valor1 in range(75, 176, 2):  # De 75 a 175 con paso 2
                    for decima in range(10):
                        valor_decimal = valor1 + (decima / 10.0)  # Combinar valor entero con décimas
                        self.valor2()
                        # time.sleep(0.1)  # Pausa de 0.1 segundos entre cambios de valor
                for valor2 in range(175, 74, -2):  # De 175 a 75 con paso -2
                    for decima in range(10):
                        valor_decimal = valor2 + (decima / 10.0)  # Combinar valor entero con décimas

                        self.write()
                        # time.sleep(0.1)  # Pausa de 0.1 segundos entre cambios de valor

            print("Índice del dispositivo actual:", self.dispositivo_index)
            self.dispositivo_index = (self.dispositivo_index + 1) % len(self.devices)

if __name__ == '__main__':
    try:
        node = client_node()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
