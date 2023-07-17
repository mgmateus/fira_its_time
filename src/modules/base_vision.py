#!/usr/bin/env python3
import rospy
import cv2
import math

from std_msgs.msg import Bool, Float64, String

class BaseVision:
    def __init__(self) -> None:
        self.__passed_direction = 'N' #salvar quando ler o QR
        self.__current_direction = None

        self.distance_to_qr = None
        self.qr_detected = None
        self.qr_data = None

        self._init_subscribers()
        

    def _init_subscribers(self) -> None:
        """Start the subscribers"""
        rospy.Subscriber('/hydrone_vision/qr_detected', Bool, \
                         self._qr_detected_callback)
        
        rospy.Subscriber('/hydrone_vision/distance_to_qr', Float64, \
                         self._distance_to_qr_callback)
        
        rospy.Subscriber('/hydrone_vision/qr_data', String, \
                         self._qr_data_callback)
        

    def _qr_detected_callback(self, msg):
        self.qr_detected = msg.data

    def _distance_to_qr_callback(self, msg):
        self.distance_to_qr = msg.data

    def _qr_data_callback(self, msg): #----------
        self.qr_data = str(msg.data).split(',') #tentativa 1
        
    def is_qr_detected(self):
        return self.qr_detected
    
    def is_aligned(self):
        return True if self.distance_to_qr <= 10 else False
    
    def is_target_position(self, mission : int):
        return True if int(self.qr_data[-1]) == mission else False

    def get_angle_to_turne(self, mission : int) -> float:
        """
        
        """
        if self.__current_direction:
            self.__passed_direction = self.__current_direction 

        self.__current_direction = self.qr_data[mission-1] #'[S,S,N,N,0]'

        if (self.__passed_direction == "N" and self.__current_direction == "N") or \
           (self.__passed_direction == "S" and self.__current_direction == "S") or \
           (self.__passed_direction == "E" and self.__current_direction == "E") or \
           (self.__passed_direction == "W" and self.__current_direction == "W"):

           return 0.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "E") or \
             (self.__passed_direction == "S" and self.__current_direction == "W") or \
             (self.__passed_direction == "E" and self.__current_direction == "S") or \
             (self.__passed_direction == "W" and self.__current_direction == "N"):
           
           return -90.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "W") or \
             (self.__passed_direction == "S" and self.__current_direction == "E") or \
             (self.__passed_direction == "E" and self.__current_direction == "N") or \
             (self.__passed_direction == "W" and self.__current_direction == "S"):
           
           return 90.0 
        
        if (self.__passed_direction == "N" and self.__current_direction == "S") or \
             (self.__passed_direction == "S" and self.__current_direction == "N") or \
             (self.__passed_direction == "E" and self.__current_direction == "W") or \
             (self.__passed_direction == "W" and self.__current_direction == "E"):
           
           return 180.0