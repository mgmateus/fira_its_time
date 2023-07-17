#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import json

from jetson_utils import *
from pyzbar.pyzbar import decode
from datetime import datetime



from std_msgs.msg import Bool, Float64, String

class Vision:
    def __init__(self) -> None:
        self._init_publishers()

    def _init_publishers(self) -> None:
        """Start the publishers"""
        self.__publisher_qr_detected = rospy.Publisher('/hydrone_vision/qr_detected', \
                                                          Bool, \
                                                          queue_size=10)
        self.__publisher_distance_to_qr = rospy.Publisher('/hydrone_vision/distance_to_qr', \
                                                          Float64, \
                                                          queue_size=10)
        
        self.__publisher_qr_data = rospy.Publisher('/hydrone_vision/qr_data', \
                                                          String, \
                                                          queue_size=10)
    

    def _gstreamer_pipeline(self):
        # ASSIGN CAMERA ADRESS to DEVICE HERE!
        pipeline = " ! ".join(["v4l2src device=/dev/video",
                            "video/x-raw, width=640, height=480, framerate=30/1",
                            "videoconvert",
                            "video/x-raw, format=(string)BGR",
                            "appsink"
                            ])

        # Sample pipeline for H.264 video, tested on Logitech C920
        h264_pipeline = " ! ".join(["v4l2src device=/dev/video0",
                                    "video/x-h264, width=640, height=640, framerate=7/1, format=H264",
                                    "avdec_h264",
                                    "videoconvert",
                                    "video/x-raw, format=(string)BGR",
                                    "appsink sync=false"
                                    ])

        return h264_pipelinelocal_position
    
        
    def qr_read(self, rasp_cam : bool = False, visualize : bool = False):
        camera = gstCamera(1280, 720, "/dev/video1")
        display = glDisplay()

        while not rospy.is_shutdown(): 
            if display.IsOpen():
                frame, w, h = camera.CaptureRGBA()
                cuda_img = cudaImage(width=w, height=h, format='rgba32f')
                cudaMemcpy(cuda_img, frame)

                bgr_img = cudaAllocMapped(width=1280,
                            height=720,
                            format='bgr8')
            
                cudaConvertColor(frame, bgr_img)
                cudaDeviceSynchronize()

                array = np.ones(frame.shape, np.float32)
                cv_img = np.add(frame, array)

                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

                qr_detected = decode(cv_img)

                imgHeight, imgWidth = cv_img.shape

                centerImgX = int(imgWidth/2)
                centerImgY = int(imgHeight/2)   
                
                #cudaDrawCircle(cuda_img, (centerImgX, centerImgY), 2, (255, 0, 0, 200))

                if qr_detected:
                    self.__publisher_qr_detected.publish(True)
                    for decodedObject in qr_detected:
                        (x,y,w,h) = decodedObject.rect
                        #cudaDrawRect(cuda_img, (x, y, x + w, y + h), (0, 0, 255, 200))

                        centerQrCodeX = int((x+w) - (w/2))
                        centerQrCodeY = int((y+h) - (h/2))

                        #cudaDrawCircle(cuda_img, (centerQrCodeX, centerQrCodeY), 2, (255, 0, 0, 200))

                        distanceX = centerQrCodeX - centerImgX
                        distanceY = centerQrCodeY - centerImgY

                        distance_to_sent = Float64()
                        distance_to_sent.data = np.sqrt(distanceX**2 + distanceY**2)
                        self.__publisher_distance_to_qr.publish(distance_to_sent)


                        #cudaDrawLine(cuda_img, (centerImgX,centerImgY), (centerQrCodeX,centerQrCodeY), (255,0,0,200), 10)

                        
                        barCode = str(decodedObject.data.decode("utf-8"))
                        barCode = barCode.replace('[', '')
                        barCode = barCode.replace(']', '')

                        string_to_sent = String()
                        string_to_sent.data = barCode
                        self.__publisher_qr_data.publish(string_to_sent) ####### 'S,S,N,N,0' ----> [S,S,N,N,0]

                else:
                    self.__publisher_qr_detected.publish(False)
                
                
                if visualize:
                    display.RenderOnce(cuda_img, w, h)

    


if __name__ == '__main__':
    rospy.init_node('hydrone_vision')

    vision = Vision()
    vision.qr_read(visualize=False)
    