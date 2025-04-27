#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32
import random
from assignments.msg import ErrorMessage

from assignments.srv import Speaker, SpeakerResponse

error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)

def speaker_callback(req):
    rospy.loginfo(f"Received a message: {req.message}")
    
    num = random.randint(1, 100)
    
    if(num < 10):
        error_pub.publish(4)
        rospy.loginfo("speaker failed")
        success = False
    else:
        # Reproduce the message
        success = True
    rospy.sleep(5)

    return SpeakerResponse(success)

def speaker_service():
    rospy.init_node('speaker_service')
    service = rospy.Service('/speaker', Speaker, speaker_callback)
    rospy.Subscriber('/error_message', ErrorMessage, error_callback)

    rospy.loginfo("Speaker service is ready!")
    rospy.spin()

def error_callback(msg):
        error = msg
        if error.id_component == 8:
            rospy.logerr(f"Received an error from the error handler: {error}")

if __name__ == '__main__':
    speaker_service()