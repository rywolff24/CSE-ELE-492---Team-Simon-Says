from datetime import datetime
import argparse
from threading import Thread
from multiprocessing import Process
import cv2
import mediapipe as mp
import math
import numpy as np
import serial
import time
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False
    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()
                
    def stop(self):
        self.stopped = True

class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("MediaPipe Pose", cv2.flip(self.frame, 0))
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

    def stop(self):
        self.stopped = True
        
class Landmarks:
    def __init__(self, frame, results):
        self.frame = frame
        self.results = results
    def start(self):
        Thread(target=self.show, args=()).start()
        return self
    def show(self):
        mp_drawing.draw_landmarks(
              self.frame,
              self.results.pose_landmarks,
              mp_pose.POSE_CONNECTIONS,
              landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())   

def getAngle(shoulder, elbow, wrist, hip):
    
    x1 = shoulder[0]
    y1 = shoulder[1]
    
    x2 = elbow[0]
    y2 = elbow[1]
    
    x3 = wrist[0]
    y3 = wrist[1]
    
    x4 = hip[0]
    y4 = hip[1]
    
    #length of shoulder to elbow
    a1 = math.sqrt(((x2-x1)**2) + ((y2-y1)**2))
    
    #length of elbow to wrist
    a2 = math.sqrt(((x3-x2)**2) + ((y3-y2)**2))
    
    #length of shoulder to hip
    a3 = math.sqrt(((x1-x4)**2) + ((y1-y4)**2))
    
    r1 = math.sqrt(((x3-x1)**2) + (((y3-y1)**2)))
    r2 = math.sqrt(((x2-x4)**2)+(((y2-y4)**2)))
    
    elbowAngle = np.arccos(((a1**2) + (a2**2) - (r1**2)) / (2*(a1*a2)))
    shoulderAngle = np.arccos(((a1**2) + (a3**2) - (r2**2)) / (2*(a1*a3)))
    
    return elbowAngle, shoulderAngle

def threadBoth(source=0):
    """
    Dedicated thread for grabbing video frames with VideoGet object.
    Dedicated thread for showing video frames with VideoShow object.
    Main thread serves only to pass frames between VideoGet and
    VideoShow objects/threads.
    """
    
    video_getter = VideoGet(source).start()
    width = cv2.CAP_PROP_FRAME_WIDTH
    height = cv2.CAP_PROP_FRAME_HEIGHT
     
    video_shower = VideoShow(video_getter.frame).start()
    
    upperBound = (width/2) + 0.2
    lowerBound = (width/2) - 0.2
    
    x = 5
    i = 0
    leftElbowAngles = []
    leftShoulderAngles = []
    rightElbowAngles = []
    rightShoulderAngles = []
    angles = ["180,", "0,", "0,", "0,", "0,", "160,", "70,"] # l elbow, l shoulder, l shoulder rotate, r elbow, r shoulder
    
    arduino = serial.Serial(port='/dev/ttyACM1', baudrate=15200)
    centerX = width/2
    prevAngle = 70
    z = 0
    
    with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.4,
        enable_segmentation=True) as pose:
        
        while True:
            start1 = time.time()
            if video_getter.stopped or video_shower.stopped:
                video_shower.stop()
                video_getter.stop()
                break

            frame = video_getter.frame
            results = pose.process(frame)
            frame.flags.writeable = True
            
            landMarks = Landmarks(frame, results).start()
            
            video_shower.frame = frame
            
            if not results.pose_landmarks:
                continue
            
            noseX = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * width
            
            ## Left Arm ##
            lx1 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x * width
            ly1 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y * height
            lx2 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].x * width
            ly2 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW].y * height
            lx3 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * width
            ly3 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * height
            lx4 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].x * width
            ly4 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].y * height
            
            lshoulder = [lx1, ly1]
            lelbow = [lx2, ly2]
            lwrist = [lx3, ly3]
            lhip = [lx4, ly4]
                
            larmDown = False
                
            lelbowAngle, lshoulderAngle = getAngle(lshoulder,
                                                   lelbow,
                                                   lwrist,
                                                   lhip)
            
            if len(leftElbowAngles) != 3:
                leftElbowAngles.append(np.degrees(lelbowAngle))
            if len(leftElbowAngles) == 3:
                first = leftElbowAngles[0]
                last = leftElbowAngles[-1]
                leftElbowAngles.clear()
                if abs(last - first) >= 5:
                    last = round(last) - 30
                    lastStr = str(last) + ","
                    angles[0] = lastStr
                    
            if len(leftShoulderAngles) != 3:
                leftShoulderAngles.append(np.degrees(lshoulderAngle))
            if len(leftShoulderAngles) == 3:
                first = leftShoulderAngles[0]
                last = leftShoulderAngles[-1]
                leftShoulderAngles.clear()
                if abs(last - first) >= 5:
                    last = round(last) - 30
                    lastStr = str(last) + ","
                    angles[1] = lastStr
    
            if (lwrist[1] < lelbow[1]):
                if(not(np.degrees(lshoulderAngle) >= 0 and np.degrees(lshoulderAngle) <= 50) and np.degrees(lelbowAngle) >= 30):
                         larmDown = True
                         
            if larmDown:
                angles[2] = '160,'
            else:
                angles[2] = '0,'  
            
            ## Right Arm ##
            rx1 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * width
            ry1 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * height
            rx2 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x * width
            ry2 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y * height
            rx3 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * width
            ry3 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * height
            rx4 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].x * width
            ry4 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].y * height
            
            rshoulder = [rx1, ry1]
            relbow = [rx2, ry2]
            rwrist = [rx3, ry3]
            rhip = [rx4, ry4]
            
            rarmDown = False
            
            relbowAngle, rshoulderAngle = getAngle(rshoulder,
                                                   relbow,
                                                   rwrist,
                                                   rhip)
            
            if len(rightElbowAngles) != 3:
                rightElbowAngles.append(np.degrees(relbowAngle))
            if len(rightElbowAngles) == 3:
                first = rightElbowAngles[0]
                last = rightElbowAngles[-1]
                rightElbowAngles.clear()
                if abs(last - first) >= 5:
                    last = round(abs(180 - last)) + 5 
                    lastStr = str(last) + ","
                    angles[3] = lastStr
                    
            if len(rightShoulderAngles) != 3:
                rightShoulderAngles.append(np.degrees(rshoulderAngle))
            if len(rightShoulderAngles) == 3:
                first = rightShoulderAngles[0]
                last = rightShoulderAngles[-1]
                rightShoulderAngles.clear()
                if abs(last - first) >= 5:
                    last = round(last)
                    lastStr = str(last) + ","
                    angles[4] = lastStr
                    
            if (noseX > upperBound) or (noseX < lowerBound):
                if(centerX < noseX): # move to the left
                    angle = prevAngle + 5
                    angle = str(round(angle))
                    angles[6] = angle
                else: # move to the right
                    angle = prevAngle - 5
                    angle = str(round(angle))
                    angles[6] = angle
                prevAngle = int(angle)
                
            if (rwrist[1] < relbow[1]):
                if(not(np.degrees(rshoulderAngle) >= 0 and np.degrees(rshoulderAngle) <= 50) and np.degrees(relbowAngle) >= 50):
                         rarmDown = True
                
            if rarmDown:
                angles[5] = '0,'
            else:
                angles[5] = '160,'    
            
            if z >= 5:
                anglesStr = angles[0] + angles[1] + angles[2] + angles[3] + angles[4] + angles[5] + angles[6]
                print(anglesStr)
                arduino.write((anglesStr).encode())
                reachedPos = str(arduino.readline())
                
            else: z += 1
            
threadBoth()
