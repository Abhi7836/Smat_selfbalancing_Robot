import cv2
import numpy as np
import serial
import cv2.aruco as aruco
import time
font=cv2.FONT_HERSHEY_SIMPLEX
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters= aruco.DetectorParameters_create()
cap=cv2.VideoCapture(0)
arduino = serial.Serial('COM8',9600)
def arucodect(image):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        corners,ids,rejected=aruco.detectMarkers(image=gray,dictionary=aruco_dict,parameters=parameters)
        if len(corners):
                x=int((corners[0][0][0][0]+corners[0][0][1][0])/2)
                y=int((corners[0][0][0][1]+corners[0][0][2][1])/2)
                aruco.drawDetectedMarkers(image,corners)
                cv2.putText(image,str(ids[0][0]), (x,y), font, 1, (0,0,255), 2, cv2.LINE_AA)
        return image,ids
def motion(ID):
        if ID:
                if ID[0][0]==1:
                        arduino.write(b'1')
                        print('ball')

                if ID[0][0]==8:
                        arduino.write(b'8')
                        print('8')
        if ID is None:
                print('no id')
               # arduino.write(b'n')
                
if __name__ == "__main__":
        while(True):
                ret,image=cap.read()
                image=cv2.resize(image,(640,360))
                img,ID=arucodect(image)
                cv2.imshow('image',img)
                motion(ID)
                k=cv2.waitKey(1)&0xFF
                if k==27:
                        break
        cap.release()
        cv2.destroyAllWindows()
        
