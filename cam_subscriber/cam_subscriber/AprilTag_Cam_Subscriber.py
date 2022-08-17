# Import ROS2 subscriber requisites
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage

# Import Message Filter things
import message_filters
from message_filters import TimeSynchronizer, Subscriber

# Import necessary OpenCV stuff
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv


class cameraFeed(Node):

    def __init__(self):
        super().__init__('Detection_GUI')
        self.subscription = self.create_subscription( 
            TFMessage,
            'tf',
            self.tf_callback,
            10)
        self.subscription
        
        # Convert ROS to OpenCV image
        self.br = CvBridge() 
        self.image_sub = Subscriber(self, Image, "camera/image")
        self.tag_sub = Subscriber(self, AprilTagDetectionArray, "apriltag/detections")
        queue_size = 30
        self.ts = TimeSynchronizer([self.image_sub, self.tag_sub], queue_size)
        self.ts.registerCallback(self.synchronized_callback)
        
    def tf_callback(self, TFMessage):
        print(TFMessage)

        
    def synchronized_callback(self, image_sub, tag_sub):
        current_frame = self.br.imgmsg_to_cv2(image_sub) # Convert ROS message to OpenCV message
        
        # AprilTag Detection
        
        for det in tag_sub.detections:

            # Find tag info, print it to GUI
            # Convert the int to a string, openCV doesn't put text from integers
            tagID, tagFamily= str(det.id), str(det.family) 
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(current_frame, 'Tag ID: ', (10, 300), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA) 
            cv.putText(current_frame, tagID, (10, 350), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            cv.putText(current_frame, 'Tag Family: ', (10, 425), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            cv.putText(current_frame, tagFamily, (10, 475), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            
            # Corners
            ptB = (int(det.corners[0].x), int(det.corners[0].y))
            ptC = (int(det.corners[1].x), int(det.corners[1].y))
            ptD = (int(det.corners[2].x), int(det.corners[2].y))
            ptA = (int(det.corners[3].x), int(det.corners[3].y))
            
            # Center
            cX, cY = int(det.centre.x), int(det.centre.y)
            
            # Draw the box around the tag
            cv.line(current_frame, ptA, ptB, (0, 255, 0), 2)
            cv.line(current_frame, ptB, ptC, (0, 255, 0), 2)
            cv.line(current_frame, ptC, ptD, (0, 255, 0), 2)
            cv.line(current_frame, ptD, ptA, (0, 255, 0), 2)
            
            # Draw the center circle
            cv.circle(current_frame, (cX, cY), 5, (0, 255, 0), 3)

        cv.imshow("taggedFrame", current_frame)
        
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    gui_subscriber = cameraFeed()
    rclpy.spin(gui_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

