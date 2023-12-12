#!/usr/bin/env python3
# -- coding: utf-8 --

import os
import sys
import json
import queue
import vosk
import sounddevice as sd
from mmap import MAP_SHARED

import rospy
import rospkg
from ros_vosk.msg import speech_recognition
from std_msgs.msg import String, Bool

import vosk_ros_model_downloader as downloader
import datetime
class vosk_sr():
    def __init__(self):
        model_name = rospy.get_param('vosk/model', "vosk-model-es-0.42")

        rospack = rospkg.RosPack()
        rospack.list()
        package_path = rospack.get_path('ros_vosk')
        
        models_dir = os.path.join(package_path, 'models')
        model_path = os.path.join(models_dir, model_name)
        
        
        if not rospy.has_param('vosk/model'):
            rospy.set_param('vosk/model', model_name)

        self.tts_status = False
        self.pub_final = rospy.Publisher('speech_recognition/final_result',String, queue_size=0)
        self.rate = rospy.Rate(1000)
        rospy.on_shutdown(self.cleanup)
        self.msg = speech_recognition()
        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        device_info = sd.query_devices(self.input_dev_num, 'input')        
        self.samplerate = int(device_info['default_samplerate'])
        rospy.set_param('vosk/sample_rate', self.samplerate)
        self.model = vosk.Model(model_path)
    
    def cleanup(self):
        rospy.logwarn("Shutting down VOSK speech recognition node...")
    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        #self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
        #print("comeinza reconocimeito"+self.current_time)

    def speech_recognize(self):    
        try:
            with sd.RawInputStream(samplerate=self.samplerate, blocksize=4000, device=self.input_dev_num, dtype='int16',
                               channels=1, callback=self.stream_callback):
                rospy.logdebug('Started recording')
                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                print("Vosk is ready to listen!")
                isRecognized = False
        
                while not rospy.is_shutdown():
                    data = self.q.get()
                    if rec.AcceptWaveform(data):
                        self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
                        print("reconoce bloque"+self.current_time)
                        result = rec.FinalResult()
                        diction = json.loads(result)
                        lentext = len(diction["text"])
                        if lentext > 2:
                            result_text = diction["text"]
                            rospy.loginfo(result_text)
                            isRecognized = True
                        else:
                            isRecognized = False
                        rec.Reset()

                    if (isRecognized is True):
                        self.msg.isSpeech_recognized = True
                        #self.current_time = datetime.datetime.now().strftime("%M:%S,%f")
                        #print("publica: "+self.current_time)
                        self.msg.time_recognized = rospy.Time.now()
                        self.msg.final_result = result_text
                        self.msg.partial_result = "unk"
                        self.pub_final.publish(result_text)
                        self.q.queue.clear()
                        isRecognized = False
                                
        except Exception as e:
            exit(type(e)._name_ + ': ' + str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the VOSK speech recognition node...")
            rospy.sleep(1)
            print("node terminated")

if __name__ == '__main__':
    try:
        rospy.init_node('vosk', anonymous=False)
        rec = vosk_sr()
        rec.speech_recognize()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)
        print("node terminated")