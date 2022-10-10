#!/usr/bin/env python3

from ast import arg
from selectors import SelectSelector
import rospy
import time
from robot_control.msg import  NavigationTaskAction,NavigationTaskResult,NavigationTaskFeedback
from robot_control.msg import DroneAction,DroneResult,DroneFeedback

import actionlib
from sensor_msgs.msg import CompressedImage

class NavigationAction():
    _feedback = NavigationTaskFeedback()
    _result = NavigationTaskResult()
    _lastImage = CompressedImage()
    
    def __init__(self,*args) -> None:
        
        if len(args)>=1:
            if isinstance(args[0],str) and (str(args[0]).find("camera")!=-1):

                # init the action server
                self._as = actionlib.SimpleActionServer("/action_server", DroneAction, self.imgCallback, True)
                self._as.start()
                
                # connect to the drone front camera
                self._camera = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, self.cameraCallback)
                
                self._result.allPictures = []
        else:
            self._as = actionlib.SimpleActionServer("/action_server", NavigationAction, self.actioncallback, True)
            self._as.start()

    def actioncallback(self,goal):
        rate=rospy.Rate(1)
        success=True
        for item in range(1,goal.unit):
            
            if self._as.is_preempt_requested():
                rospy.loginfo("canceling action goal")
                self._as.set_preempted()
                success=False
                break
            self._feedback.feedback=str(item*100/goal.unit) # set the feedback
            self._as.publish_feedback(self._feedback) # publish the feedback

        rospy.loginfo("finishing action goal")
        if success:
            self._result.result=f"{goal.unit} items have been taken"
            self._as.set_succeeded(self._result)
        else:
            self._result.result=f"{goal.unit} items have not been taken"
            self._as.set_aborted(self._result)

    def cameraCallback(self, msg):
        
        self._lastImage = msg
        
        
    def imgCallback(self, goal):
        r = rospy.Rate(1)
        
        success = True
        
        for i in range(1, goal.nseconds):
            
            # check if there are a preemption request
            if self._as.is_preempt_requested():
                rospy.loginfo('Cancelling image taking action server')
                self._as.set_preempted()
                success = False
                break
            
            self._feedback.lastImage = self._lastImage
            self._result.allPictures.append(self._lastImage)
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            
            r.sleep()
            
        if success:
            rospy.loginfo('Finishing.All the images have been taken')
            self._as.set_succeeded(self._result)
            
if __name__ == '__main__':
    
    rospy.init_node('navaction_server')
    NavigationAction()
    rospy.spin()