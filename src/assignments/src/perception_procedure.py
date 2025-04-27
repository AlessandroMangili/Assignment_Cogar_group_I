#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String, Int32
from assignments.msg import ErrorMessage

class PerceptionSubsystem:
    def __init__(self):
        rospy.init_node('perception_subsystem')
        
        self.rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
        self.error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)
        self.detection_pub = rospy.Publisher('/object_detected', Bool, queue_size=10)
        self.tracking_pub = rospy.Publisher('/object_tracked', Bool, queue_size=10)

        rospy.Subscriber('/error_message', ErrorMessage, self.error_callback)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.run_loop)
        rospy.loginfo("Simplified Perception System Ready")

    def error_callback(self, msg):
        error = msg
        if error.id_component == 9:
            rospy.logerr(f"Received an error from the error handler: {error}")

    def run_loop(self, event):        
        rgb_msg = Image()
        depth_msg = PointCloud2()
        rgb_msg.header.stamp = rospy.Time.now()
        depth_msg.header.stamp = rospy.Time.now()
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        
        if random.random() > 0.3:
            self.detection_pub.publish(True)
            self.tracking_pub.publish(random.random() > 0.2)
        else:
            self.detection_pub.publish(False)
            self.tracking_pub.publish(False)

if __name__ == '__main__':
    try:
        PerceptionSubsystem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
