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

#Table making stuff
import csv
from datetime import datetime
from datetime import date
import os.path


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
        self.tag_sub = Subscriber(self, AprilTagDetectionArray, "/detections")
        queue_size = 30
        self.ts = TimeSynchronizer([self.image_sub, self.tag_sub], queue_size)
        self.ts.registerCallback(self.synchronized_callback)
        self._win_name = "taggedImage"
        cv.namedWindow(self._win_name)
        
        #Initialize as blank variables
        
        self.equalX = ""
        self.equalY = ""
        self.equalZ = ""
        self.tagID = ""
        self.tagFamily = ""
        self.RotationW = ""
        self.RotationX = ""
        self.RotationY = ""
        self.RotationZ = ""
        self.TranslationX = ""
        self.TranslationY = ""
        self.TranslationZ = ""
        
        self.dist_coeffs = np.zeros((4,1))
        
        #I'm not entirely sure what Ben was doing with this
        self.AT_world = (0, 279.4, -762)
        
        #This will come directly out of the camera calibration package once that is running
        self.camera_matrix = np.array([[1166.666667, 0.0, 320.0], [0.0, 1166.666667, 256.0], [0.0, 0.0, 1.0]])
        
        #This should ultimately be grabbed from some other intrinsic, not hardcoded.
        self.tag_world_xyz = np.array([(0 + self.AT_world[0],     0+ self.AT_world[1],     self.AT_world[2]), 
                          (0 + self.AT_world[0],     152.4+ self.AT_world[1], self.AT_world[2]), 
                          (152.4 + self.AT_world[0], 152.4+ self.AT_world[1], self.AT_world[2]),
                          (152.4 + self.AT_world[0], 0 + self.AT_world[1],    self.AT_world[2])
                        ],dtype="double")

        
    def tf_callback(self, TFMessage):
        for mes in TFMessage.transforms:
            xyz = str(mes).split("Vector3(")[1].split(")")[0] #x=0.0, y=0.0, z=0.0
            splitUp = xyz.split(", ")
            self.equalX = "x=" + str((float(splitUp[0].split("=")[1]) * 100.0))
            self.equalY = "y=" + str((float(splitUp[1].split("=")[1]) * 100.0))
            self.equalZ = "z=" + str((float(splitUp[2].split("=")[1]) * 100.0))

        
    def synchronized_callback(self, image_sub, tag_sub):
        current_frame = self.br.imgmsg_to_cv2(image_sub) # Convert ROS message to OpenCV message
        
       # print("In sync callback")
        # AprilTag Detection
        for det in tag_sub.detections:
            # Find tag info, print it to GUI
            # Convert the int to a string, openCV doesn't put text from integers
            self.tagID, self.tagFamily = str(det.id), str(det.family)
           # print("In detections")
          #  print("Tag ID: ", self.tagID)
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(current_frame, 'Tag ID: ', (10, 300), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA) 
            cv.putText(current_frame, self.tagID, (10, 350), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            cv.putText(current_frame, 'Tag Family: ', (10, 425), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            cv.putText(current_frame, self.tagFamily, (10, 475), 
                       font, 1, (0, 255, 0), 1, cv.LINE_AA)
            
            
            #Corners
            ##Note: there is a _F and a _I extension because the lines and putText functions do not like tuples,
            ##they must be integers
            #Bottom-left
            ptB_F = (float(det.corners[0].x), float(det.corners[0].y))
            #Bottom-right
            ptC_F = (float(det.corners[1].x), float(det.corners[1].y))
            #Top-right
            ptD_F = (float(det.corners[2].x), float(det.corners[2].y))
            #Top-left
            ptA_F = (float(det.corners[3].x), float(det.corners[3].y))
            
            ptB_I = (int(det.corners[0].x), int(det.corners[0].y))
            ptC_I = (int(det.corners[1].x), int(det.corners[1].y))
            ptD_I = (int(det.corners[2].x), int(det.corners[2].y))
            ptA_I = (int(det.corners[3].x), int(det.corners[3].y))
            
            # Center
            cX, cY = int(det.centre.x), int(det.centre.y)
            
            # Draw the box around the tag
            cv.line(current_frame, ptA_I, ptB_I, (0, 255, 0), 2)
            cv.line(current_frame, ptB_I, ptC_I, (0, 255, 0), 2)
            cv.line(current_frame, ptC_I, ptD_I, (0, 255, 0), 2)
            cv.line(current_frame, ptD_I, ptA_I, (0, 255, 0), 2)
            
            #Draw Ben's bounding - puts the tag_world_coords on each corner
            cv.putText(current_frame, f"1 {self.tag_world_xyz[0]}", ptB_I, cv.FONT_HERSHEY_SIMPLEX, 0.3, 255)
            
            cv.putText(current_frame, f"2 {self.tag_world_xyz[1]}", ptC_I, cv.FONT_HERSHEY_SIMPLEX, 0.3, 255)
            
            cv.putText(current_frame, f"3 {self.tag_world_xyz[2]}", ptD_I, cv.FONT_HERSHEY_SIMPLEX, 0.3, 255)
            
            cv.putText(current_frame, f"4 {self.tag_world_xyz[3]}", ptA_I, cv.FONT_HERSHEY_SIMPLEX, 0.3, 255)
            #print(det.corners)
            
            #This is made into a contiguous numpy array explictly because solvePnPRansac requires it, and the
            #message type is not a numpy array by default
            newCorners = [ptB_F, ptC_F, ptD_F, ptA_F]
            newCorners = np.ascontiguousarray(newCorners)
            print(newCorners)
            
            #RANSAC is a random sample consensus - basically it is an iterative method to estimate parameters of a model using random sampling of observed data. The inliers and outliers are then voted on. The algorithm has two assumptions. Nosiy data will not consistently vote for a single model scheme. Second, there are enough features to agree on a good model (not a lot of missing data)
            
            #The PnP algorithm (a.k.a the Perspective-n-Point algorithm) estimates an object pose given a set of object points, corresponding image projections, camera matrix, and distortion coefficients. The function finds a pose that minimizes reprojection error (the sum of squared distances between objerved projections and projected object points).
            #Ransac makes this function resistant to outliers
            success, rvec, tvec, inliers = cv.solvePnPRansac(self.tag_world_xyz, newCorners, self.camera_matrix, self.dist_coeffs)
            
            #The following converts the object position to camera position in the world frame
            print("PnPRansac rvec: ", rvec)
            #Invert signs and put all data indices on single axis
            rvec = -np.squeeze(rvec)
            print("Squeezed rvec: ", rvec)
            #Similarly, put all data on single axis (but don't invert signs)
            tvec = np.squeeze(tvec)
            print("Tvec: ", tvec)
            #Converts a rotation matrix to a rotation vector or vise versa.
            #In this case, converts the translation vector to a translation matrix             
            R, _ = cv.Rodrigues(rvec)
            print("Rodrigues: ", R)
            #Transpose the matrix 
            R_inv = np.transpose(R)
            print("Inverse Rodrigues: ", R_inv)
            #Matrix Multiplication
            RT = np.matmul(R_inv, tvec) * -1
            print("RT: ", RT)
            #Put 1s on the diagonal and 0s elsewhere
            T = np.eye(4)
            print("T: ", T)
            
            T[:3, :3] = R_inv
            print("Inverted T: ", T)
            T[:3, 3] = np.squeeze(RT)
            print("Squeezed T: ", T)
            
            print("Camera Location: ", np.array(np.matmul(T, np.array([0,0,0,1]))[0:3]))
            
            # Draw the center circle
            cv.circle(current_frame, (cX, cY), 5, (0, 255, 0), 3)

        cv.imshow(self._win_name, current_frame)
        
        
        cv.waitKey(1)
        
        #Start writing data to a csv file
        current_datetime = datetime.now()
        str_current_datetime = str(current_datetime) #Convert datetime object to a string
        current_date = date.today()
        str_current_date = str(current_date)
        
        #Start making a csv
        filename = str_current_date+".csv"
      #  save_path = '/Detections/'
       # completeName = os.path.join(save_path, filename + ".csv")
        
        fields = ['Date & Time', 'Tag ID', 'X-Coordinate', 'Y-Coordinate', 'Z-Coordinate'] #Set the columns   
        
        rows = [[str_current_datetime, self.tagID, self.equalX, self.equalY, self.equalZ]] #set the rows
        
        #This automatically closes the file once complete
        with open(filename, 'a+') as csvfile:
            csvwriter = csv.writer(csvfile) #create a writer object
            csvwriter.writerow(fields) #write the columns
            csvwriter.writerows(rows) #write the rows
           

def main(args=None):
    rclpy.init(args=args)
    gui_subscriber = cameraFeed()
    rclpy.spin(gui_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

