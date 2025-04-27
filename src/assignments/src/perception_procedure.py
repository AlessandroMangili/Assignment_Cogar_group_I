#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray, Detection3DArray
from std_msgs.msg import Bool
from assignments.msg import PerceptionCommand, PerceptionStatus

class RGBCamera:
    def __init__(self):
        self.rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
        
    def publish_data(self):
        """Publish RGB-D data"""
        rgb_msg = Image()
        depth_msg = PointCloud2()
        
        rgb_msg.header.stamp = rospy.Time.now()
        depth_msg.header.stamp = rospy.Time.now()
        
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        rospy.loginfo_once("Published RGB-D data")

class ObjectRecognizer:
    def __init__(self):
        self.detection_pub = rospy.Publisher('/object_recognition/detections', Detection2DArray, queue_size=10)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image)
        
    def process_image(self, rgb_image):
        """Process incoming images for object detection"""
        detections = Detection2DArray()
        detections.header.stamp = rospy.Time.now()
        
        if random.random() > 0.3:  # 70% chance of detecting objects
            self.detection_pub.publish(detections)
            rospy.loginfo_once("Published object detections")

class ObjectTracker:
    def __init__(self):
        self.tracking_pub = rospy.Publisher('/object_tracking/tracks', Detection3DArray, queue_size=10)
        self.detection_sub = rospy.Subscriber('/object_recognition/detections', Detection2DArray, self.track_objects)
        self.pointcloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.store_pointcloud)
        self.current_pointcloud = None
        
    def store_pointcloud(self, pointcloud):
        """Store the latest pointcloud data"""
        self.current_pointcloud = pointcloud
        
    def track_objects(self, detections):
        """Track detected objects"""
        if self.current_pointcloud is None:
            return
            
        tracks = Detection3DArray()
        tracks.header.stamp = rospy.Time.now()
        
        if detections.detections:
            self.tracking_pub.publish(tracks)
            rospy.loginfo_once("Published object tracks")

class PerceptionSubsystem:
    def __init__(self):
        rospy.init_node('perception_subsystem')
        
        # Initialize components
        self.camera = RGBCamera()
        self.recognizer = ObjectRecognizer()
        self.tracker = ObjectTracker()
        self.active = False
        
        # Create command subscriber and status publisher
        self.command_sub = rospy.Subscriber('/perception_command', PerceptionCommand, self.command_callback)
        self.status_pub = rospy.Publisher('/perception_status', PerceptionStatus, queue_size=10)
        
        # Timer for periodic operation
        self.timer = rospy.Timer(rospy.Duration(0.1), self.run_pipeline)
        
        rospy.loginfo("Perception Subsystem initialized (Pub/Sub version)")

    def command_callback(self, msg):
        """Handle perception commands"""
        if msg.enable and not self.active:
            self.active = True
            status = PerceptionStatus()
            status.active = True
            status.message = "Perception pipeline activated"
            self.status_pub.publish(status)
            rospy.loginfo("Starting perception pipeline")
            
        elif not msg.enable and self.active:
            self.active = False
            status = PerceptionStatus()
            status.active = False
            status.message = "Perception pipeline deactivated"
            self.status_pub.publish(status)
            rospy.loginfo("Stopping perception pipeline")

    def run_pipeline(self, event):
        """Main processing loop"""
        if not self.active:
            return
            
        # Publish camera data
        self.camera.publish_data()
        
        # Object recognition and tracking happen via callbacks

if __name__ == '__main__':
    try:
        perception = PerceptionSubsystem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass