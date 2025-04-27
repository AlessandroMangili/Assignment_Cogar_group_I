#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from assignments.srv import CheckJointState, CheckJointStateResponse
import random
from assignments.msg import ErrorMessage

expected_joint_count = 7

def check_joint_state(req):
    """
    Service callback that checks the dimensions of the provided JointState request.
    """
    rospy.loginfo("Received joint state for comparison.")
    num = random.randint(1, 100)
    
    if(num < 20):
        rospy.loginfo("Pose unreachable")
        rospy.sleep(5)
        return CheckJointStateResponse(False)

    # Check if the dimensions of positions, velocities, and efforts match the expected joint count
    if len(req.positions) == expected_joint_count and \
       len(req.velocities) == expected_joint_count and \
       len(req.efforts) == expected_joint_count:
        rospy.loginfo("Joint state dimensions match.")
        rospy.sleep(5)
        rospy.loginfo("The arm was moved!")
        return CheckJointStateResponse(True)
    else:
        rospy.loginfo("Joint state does not match.")
        return CheckJointStateResponse(False)

def joint_state_service():
    """
    Initializes the ROS service server that checks the dimensions of the joint state.
    """
    rospy.init_node('joint_state_service', anonymous=True)
    # Create the service that listens for requests
    service = rospy.Service('/check_joint_state', CheckJointState, check_joint_state)
    rospy.Subscriber('/error_message', ErrorMessage, error_callback)
    error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)
    rospy.loginfo("Joint state service is ready!")
    rospy.spin()

def error_callback(msg):
        error = msg
        if error.id_component == 7:
            rospy.logerr(f"Received an error from the error handler: {error}")

if __name__ == '__main__':
    try:
        joint_state_service()
    except rospy.ROSInterruptException:
        pass