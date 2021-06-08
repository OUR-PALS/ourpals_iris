#!/usr/bin/env python

#Must install sox: 'sudo apt install sox'
# import necessary packages
import rospy
import cv2 as cv
import queue, threading
import numpy as np
import dlib 
from std_msgs.msg import String
from math import hypot
#import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Twist
#for sound
try:
  import os
except:
  import winsound

class commander:
    '''
    This Class will have all the servers of service-client to perform robot operations
    Robot Operations include
    moving the robot
    starting the robot
    stopping the robot

    '''
    def __init__(self):
        '''
        Initialze the services
        '''
        
        self.initialize_publishers()
      
        self.step_size=1.0
        self.linear_vel=0.14
        self.angular_vel=0.09
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.stop_move)

    def initialize_publishers(self):
        '''
        To create publishers
        for turtlesim: uncomment first line comment second
        :return:
        '''
        #self.mover_ = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        #self.mover_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.mover_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    
    def publish_motion(self, msg):
        self.mover_.publish(msg)

    def stop_move(self):
        '''
        To stop moving
        :return:
        '''
        vel_msg=Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0

        vel_msg.angular.x=0
        vel_msg.angular.y=0
        vel_msg.angular.z=0
        self.publish_motion(vel_msg)

    
    def move_forward(self):
      '''
      Move forward for 1 step
      :return:

      '''
      vel_msg=Twist()
      vel_msg.linear.x=self.linear_vel
      vel_msg.linear.y=0
      vel_msg.linear.z=0

      vel_msg.angular.x=0
      vel_msg.angular.y=0
      vel_msg.angular.z=0

      #publish motion
      self.publish_motion(vel_msg)


    def move_backward(self):
      '''
      Move forward for 1 step
      :return:

      '''
      vel_msg=Twist()
      vel_msg.linear.x=-self.linear_vel
      vel_msg.linear.y=0
      vel_msg.linear.z=0

      vel_msg.angular.x=0
      vel_msg.angular.y=0
      vel_msg.angular.z=0

      #publish motion
      self.publish_motion(vel_msg)  

    
    def move_left(self):
      '''
      Move forward for 1 step
      :return:

      '''
      vel_msg=Twist()
      vel_msg.linear.x=0
      vel_msg.linear.y=0
      vel_msg.linear.z=0

      vel_msg.angular.x=0
      vel_msg.angular.y=0
      vel_msg.angular.z=self.angular_vel

      #publish motion
      self.publish_motion(vel_msg) 

    def move_right(self):
      '''
      Move forward for 1 step
      :return:

      '''
      vel_msg=Twist()
      vel_msg.linear.x=0
      vel_msg.linear.y=0
      vel_msg.linear.z=0

      vel_msg.angular.x=0
      vel_msg.angular.y=0
      vel_msg.angular.z=-self.angular_vel

      #publish motion
      self.publish_motion(vel_msg)   

        

    




#beep sound
def beep():
  duration = 0.3  # seconds
  freq = 440  # Hz
  try:
    os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
  except:
    winsound.Beep(freq, duration)

'''To Overcome the buffer queue issue in Unix based systems
  refer: https://stackoverflow.com/questions/54460797/how-to-disable-buffer-in-opencv-camera
 '''
class VideoCapture:

  def __init__(self, name):
    self.cap = cv.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()


# using the inbuilt webcam 
cap=VideoCapture(0)

# using dlib pre trained model
detector=dlib.get_frontal_face_detector()
predictor=dlib.shape_predictor("/home/surya/ourpals_ws/src/ourpals_iris/src/shape_predictor_68_face_landmarks.dat")

def midpoint(p1,p2):
    return int((p1.x+p2.x)/2),int((p1.y+p2.y)/2)

# function that ouptuts gaze ratio
def ratio(eye_points,facial_landmarks):
    left_eye_region = np.array([(facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y),(facial_landmarks.part(eye_points[1]).x, facial_landmarks.part(eye_points[1]).y),(facial_landmarks.part(eye_points[2]).x, facial_landmarks.part(eye_points[2]).y),
                            (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y),
                            (facial_landmarks.part(eye_points[4]).x, facial_landmarks.part(eye_points[4]).y),
                            (facial_landmarks.part(eye_points[5]).x, facial_landmarks.part(eye_points[5]).y)], np.int32)
    height,width,_=frame.shape
    mask=np.zeros((height,width),np.uint8)
    cv.polylines(mask,[left_eye_region],True,255,2)
    cv.fillPoly(mask,[left_eye_region],255)
    eye=cv.bitwise_and(gray,gray,mask=mask)
    min_x = np.min(left_eye_region[:, 0])
    max_x = np.max(left_eye_region[:, 0])
    min_y = np.min(left_eye_region[:, 1])
    max_y = np.max(left_eye_region[:, 1])
    gray_eye = eye[min_y: max_y, min_x: max_x]
    _, threshold_eye = cv.threshold(gray_eye, 70, 255, cv.THRESH_BINARY)
    height, width = threshold_eye.shape
    left_side_threshold = threshold_eye[0: height, 0: int(width / 2)]
    left_side_white = cv.countNonZero(left_side_threshold)
    right_side_threshold = threshold_eye[0: height, int(width / 2): width]
    right_side_white = cv.countNonZero(right_side_threshold)
    if left_side_white==0:
        gaze_ratio=2

    elif right_side_white==0:
         gaze_ratio=5

    else: 
        gaze_ratio = left_side_white / right_side_white
    return(gaze_ratio)

# funtion for blink detection
def blink(eye_points,facial_landmarks):
    left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))
    hor_line_lenght = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    ver_line_lenght = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))
    ratio = hor_line_lenght / ver_line_lenght
    return ratio
# Continuous interations to output conditions based on function return values


    
pub = rospy.Publisher('iris_command', String, queue_size=10)
rospy.init_node('pupil_tracker', anonymous=True)
rate = rospy.Rate(10)
commandcontroller=commander()
while not rospy.is_shutdown():
    frame=cap.read()
    
    gray=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    faces=detector(gray)
    command="NA"
    for face in faces:
        #cv.imshow("Frame",gray)
        landmarks = predictor(gray, face)
        gaze_ratio_left_eye = ratio([36, 37, 38, 39, 40, 41], landmarks)
        gaze_ratio_right_eye = ratio([42, 43, 44, 45, 46, 47], landmarks)
        gaze_ratio = (gaze_ratio_right_eye + gaze_ratio_left_eye) / 2
        left_eye_ratio = blink([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = blink([42, 43, 44, 45, 46, 47], landmarks)
        blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
        if gaze_ratio <= 1:
            print("RIGHT")  
            command="RIGHT"
            commandcontroller.move_right()
        elif 1 < gaze_ratio < 3:
            print("FORWARD")
            command="FORWARD"
            commandcontroller.move_forward()
        else:
            print("LEFT")
            command="LEFT"
            commandcontroller.move_left()
        
        if blinking_ratio > 5.7:
            print("Backward")
            command="LEFT"
            commandcontroller.move_backward()
        

    pub.publish(command)    
    
    #cv.imshow("Frame",frame)
    #key=cv.waitKey(1)
    n=10
    for i in range (0,n):
      if i > n-4:
        commandcontroller.stop_move()
        beep()
      time.sleep(1)  
    #if key==ord('q'):
    #    break
cap.release()
cv.destroyAllWindows()    


