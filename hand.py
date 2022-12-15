#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp
import rospy
import math
import sys
from geometry_msgs.msg import Point
from numpy.lib.type_check import imag
from robotdesign3_2021_1.msg import CustomArray
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

def main():
  rospy.init_node("MediaPipe_node")
  hand_position = rospy.Publisher('hand_topic', CustomArray, queue_size=10)
  # For webcam input:
  cap = cv2.VideoCapture(0)
  array_points = CustomArray()
  with mp_hands.Hands(
      model_complexity=0,
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5) as hands:
    while not rospy.is_shutdown():
      success, image = cap.read()
      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue

      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      results = hands.process(image)

      # Draw the hand annotations on the image.
      image.flags.writeable = True
      image_height, image_width, _ = image.shape
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
          #必要な手の関節座標を変数に格納
          for index, landmark in enumerate(hand_landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            if index == 0:
              cx0,cy0 = landmark_x, landmark_y
            if index == 9:
              cx9,cy9 = landmark_x, landmark_y
          #取得した座標の中間点を表示
          gap1x,gap1y = (cx0+cx9)/2, (cy0+cy9)/2
          gap1x = int(gap1x)
          gap1y = int(gap1y)
          cv2.circle(image, (gap1x,gap1y), 5, (0, 255, 0), 2)
          gap1x,gap1y = gap1x-image_width/2,-(gap1y-image_height/2)

          array_points.points = []
          point_1 = Point()
          point_1.x = gap1x/3000
          point_1.y = gap1y/3000
          point_1.z = 0
          array_points.points.append(point_1)
          hand_position.publish(array_points)
          #カメラに表示
          cv2.putText(image,"gap1x:"+str(gap1x)+"gap1y:"+str(gap1y),(10,30),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)


          mp_drawing.draw_landmarks(
              image,
              hand_landmarks,
              mp_hands.HAND_CONNECTIONS,
              mp_drawing_styles.get_default_hand_landmarks_style(),
              mp_drawing_styles.get_default_hand_connections_style())
      # Flip the image horizontally for a selfie-view display.
      cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))

      if cv2.waitKey(5) & 0xFF == 27:
        break
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: 
    pass  