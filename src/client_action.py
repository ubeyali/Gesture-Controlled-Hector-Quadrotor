#! /usr/bin/env python
from model import get_model
import numpy as np
import roslib
roslib.load_manifest('hector_quadrotor_smart_drone')
import rospy
import actionlib
import time


import sys
ros_path='/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
	sys.path.remove(ros_path)

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from model import get_model
import numpy as np
import cv2



recognizer = get_model()
recognizer.load_weights("weights.hdf5")

class_indices = {0: 'down', 1: 'left', 2: 'one', 3: 'open', 4: 'right', 5: 'two', 6: 'up'}

cap = cv2.VideoCapture(0)

from hector_quadrotor_smart_drone.msg import DroneAction, DroneGoal

if __name__ == '__main__':
    rospy.init_node('drone_client')
    client = actionlib.SimpleActionClient('Drone', DroneAction)
    client.wait_for_server()
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        img2detect = frame[72:268, 402:598, :]
        img = cv2.cvtColor(img2detect, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (196, 196))
        img = img / 255.0
        gesture = recognizer.predict_classes(x=np.expand_dims(img, axis=0))
        cv2.putText(frame, "Gesture: {}".format(class_indices[gesture[0]]),
                (402, 30), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255), 2)
        cv2.rectangle(frame, (401, 71), (601, 271), (0, 255, 0), 1)
        cv2.imshow('frame', frame)
        goal = DroneGoal()
        goal.gesture_id=gesture[0]
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5.0))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

