#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool

class PerceptionSubsystem:
    def __init__(self):
        rospy.init_node('perception_subsystem')
        
        
        self.rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
        
        
        self.detection_pub = rospy.Publisher('/object_detected', Bool, queue_size=10)
        self.tracking_pub = rospy.Publisher('/object_tracked', Bool, queue_size=10)
        
        
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.run_loop)
        rospy.loginfo("Simplified Perception System Ready")

   

    def run_loop(self, event):
        
        if not self.active:
            return
            
        
        rgb_msg = Image()
        depth_msg = PointCloud2()
        rgb_msg.header.stamp = rospy.Time.now()
        depth_msg.header.stamp = rospy.Time.now()
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        
        
        if random.random() > 0.3:
            self.detection_pub.publish(True)
            
            
            self.tracking_pub.publish(random.random() > 0.2)  # 80% tracking success
        else:
            self.detection_pub.publish(False)
            self.tracking_pub.publish(False)

if __name__ == '__main__':
    try:
        PerceptionSubsystem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass