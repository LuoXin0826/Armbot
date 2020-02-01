import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
from numpy.linalg import inv
import math

D2R = 3.141592/180.0
R2D = 180.0/3.141592

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        self.calibrated = False
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.grab_click_point = np.zeros((1,2),int)
        self.place_click_point = np.zeros((1,2),int)
        self.affine = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.clickandplace = np.zeros((2,2),int)
        #self.inverseK = np.array([0, 0, 100, np.pi/2])
        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrameGray(self, img):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_Indexed8
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions

        DONE
        """
        pts1 = coord1[0:5].astype(np.float32)
        pts2 = coord2[0:5].astype(np.float32)

        pts1 = pts1.reshape(10,1) 
        

        self.A = np.array([[pts2[0][0], pts2[0][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts2[0][0], pts2[0][1], 1],
                            [pts2[1][0], pts2[1][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts2[1][0], pts2[1][1], 1],
                            [pts2[2][0], pts2[2][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts2[2][0], pts2[2][1], 1],
                            [pts2[3][0], pts2[3][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts2[3][0], pts2[3][1], 1],
                            [pts2[4][0], pts2[4][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts2[4][0], pts2[4][1], 1]])
                            
        self.results = np.matmul(np.matmul(inv(np.matmul(self.A.transpose(),self.A)),self.A.transpose()),pts1)

        self.affine = np.array([[self.results[0], self.results[1], self.results[2]],
                                [self.results[3], self.results[4], self.results[5]]])
        
        # Affine for wrold coordinates:
        self.real_world_points = np.array([[-304], [-304], [-304],[304],[304],[304],[304],[-304],[35],[0]])
        pts1 = coord1[0:5].astype(np.float32)
        
        print "Original Points = " + str(pts1)
        i = 0
        z = np.array([0.0,0.0,0.0,0.0,0.0])
        for coords in pts1:
            z[i] = self.currentDepthFrame[int(pts1[i][1])][int(pts1[i][0])]
            depth = (0.1236 * np.tan((z[i])/2842.5 + 1.1863))
            mouse = np.array([[pts1[i][0]],[pts1[i][1]],[1]])
            camera = depth*np.matmul(inv(self.camera_matrix),mouse)
            print "camera coordinates = " + str(camera)
            pts1[i][0] = camera[0]
            pts1[i][1] = camera[1]
            i += 1
        
        print "Converted Points = " + str(pts1)
        
        self.B = np.array([[pts1[0][0], pts1[0][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts1[0][0], pts1[0][1], 1],
                            [pts1[1][0], pts1[1][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts1[1][0], pts1[1][1], 1],
                            [pts1[2][0], pts1[2][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts1[2][0], pts1[2][1], 1],
                            [pts1[3][0], pts1[3][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts1[3][0], pts1[3][1], 1],
                            [pts1[4][0], pts1[4][1], 1, 0, 0, 0 ],
                            [0, 0, 0, pts1[4][0], pts1[4][1], 1]])
                            
        self.resultsB = np.matmul(np.matmul(inv(np.matmul(self.B.transpose(),self.B)),self.B.transpose()),self.real_world_points)

        self.affineB = np.array([[self.resultsB[0], self.resultsB[1], self.resultsB[2]],
                                [self.resultsB[3], self.resultsB[4], self.resultsB[5]],
                                [0,0,1]])
        


        pass


    def registerDepthFrame(self):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        DONE
        """
        self.currentDepthFrame = cv2.warpAffine(self.currentDepthFrame,self.affine,(640,480))

        pass

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        DONE
        """
        h=480
        w=640
        self.camera_matrix= np.array([[522.2261029167, 0, 307.89222213], [0,521.70431689,235.90812847], [0,0,1]])
        dist_coefs = np.array([0.256447019, -0.996096330, -0.00442416200, -0.000906615776, 1.58012486])
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix,dist_coefs,(w,h),1,(w,h))

        #undistorted = cv2.undistort(self.currentVideoFrame, self.camera_matrix, dist_coefs, None, new_camera_matrix)
        #self.currentVideoFrame = undistorted
        pass 

    def draw_gray (self, gr):
        font = cv2.FONT_HERSHEY_SIMPLEX
        lower_gray = np.array([109, 80])
        upper_gray = np.array([135, 255])
        gray = cv2.inRange(gr, lower_gray, upper_gray)
        _, contours, _= cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        i = 0
        while i < len(contours):
            cnt = contours[i]
            rect = cv2.minAreaRect(cnt)
            coord = np.array([[rect[0][0]],[rect[0][1]],[1]])
            
            if self.calibrated == False:
                cv2.putText(self.currentVideoFrame,"%.2f, %.2f" % (coord[0],coord[1]),(int(coord[0]) - 70,int(coord[1]) - 20),font, 0.5, (255,0,0),1)
            elif self.calibrated == True:
                pos = np.matmul(self.affineB.astype(float),coord.astype(float))
                cv2.putText(self.currentVideoFrame,"%.2f, %.2f" % (pos[0],pos[1]),(int(coord[0]) - 70,int(coord[1]) - 20),font, 0.5, (255,0,0),1)
               
                self.inverseK[0] = pos[0]
                self.inverseK[1] = pos[1]
                
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(self.currentVideoFrame,[box],0,(0,255,0),2)
            i = i +1

    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        rgb_frame = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)


        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
       
        self.DepthHSV[...,0] = self.currentDepthFrame
        self.DepthHSV[...,1] = 0x9F
        self.DepthHSV[...,2] = 0xFF
        self.DepthRGB = cv2.cvtColor(self.DepthHSV, cv2.COLOR_HSV2RGB)	
        self.DepthCM = cv2.cvtColor(self.DepthRGB, cv2.COLOR_RGB2GRAY)	
        cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)
        
        if self.calibrated == True:

            self.DepthCM = self.DepthCM[self.rgb_click_points[1][1]:self.rgb_click_points[0][1],self.rgb_click_points[0][0]:self.rgb_click_points[3][0]]
            top = self.rgb_click_points[1][1]
            bottom = 480 - int(self.rgb_click_points[0][1])
            left = self.rgb_click_points[0][0]
            right = 640 - int(self.rgb_click_points[3][0])
            self.DepthCM = cv2.copyMakeBorder(self.DepthCM,top,bottom,left,right,cv2.BORDER_CONSTANT,value=[255,255,255])

            self.DepthCM = cv2.GaussianBlur(self.DepthCM, (5,5),0)
            self.DepthCM = cv2.threshold(self.DepthCM, 202, 255, cv2.THRESH_BINARY)[1]
            self.DepthCM = (255-self.DepthCM)

            kernel = np.ones((3,3),np.uint8)
            Erode = cv2.erode(self.DepthCM,kernel,iterations = 2)
            Dilation = cv2.dilate(Erode,kernel,iterations = 1)

            im2, contours, hierarchy = cv2.findContours(Dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
               
            i = 0
            blocklist = []
            while i < len(contours):
                cnt = contours[i]
                rect = cv2.minAreaRect(cnt)

                if rect[1][1] > 15 and rect[1][1] < 80 and rect[1][0] > 15 and rect[1][0] < 80:
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(self.currentVideoFrame,[box],0,(0,255,0),2)
                    
                    
                    coord = np.array([[rect[0][0]],[rect[0][1]],[1]])
                    z = self.currentDepthFrame[int(coord[1])][int(coord[0])]
                    depth = 0.1236 * np.tan(z/2842.5 + 1.1863)
                    camera = depth*np.matmul(inv(self.camera_matrix),coord)
                    points = np.array([[camera[0]],[camera[1]],[1]])
                    world = np.matmul(self.affineB.astype(float),points.astype(float))
                    
                    board_depth = 0.1236 * np.tan(718/2842.5 + 1.1863)
                
                    depth = (board_depth-depth)*1000
                 
                    color = self.getcolor(rect[0][0],rect[0][1])
                    cv2.putText(self.currentVideoFrame,color,(int(rect[0][0]),int(rect[0][1]) - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),2)

                    blocklist.append([world[0], world[1], depth+70, -np.pi/2, rect[2]*D2R, color])

                i = i +1
   
            self.inverseK = np.array(blocklist)


        return self.DepthCM
  

    def getcolor(self, x, y):
        
        img = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
        
               
        hsv_pixel = np.array(hsv[int(y-3):int(y+3),int(x-3):int(x+3)])
        img_pixel = np.array(img[int(y-3):int(y+3),int(x-3):int(x+3)])

        count = 0
        average = 0
        average2 = 0
        x_value = 0
        y_value = 0
        for row in hsv_pixel:
            for column in row:
                if column[2] > 1:
                    if column[0] < 3:
                        column[0] = 180
                    if column[0] > 174:
                        column[0] = 180

                    average = average + int(column[0])
                    average2 = average2 + int(column[1])
                    count = count + 1

        average = average/count
        average2 = average2/count
          
        if 0 < average < 15:
            color = "ORANGE" 
        elif 20 < average < 35:
            color = "YELLOW"
        elif 40 < average < 90:
            color = "GREEN" 
        elif 100 < average < 140 and average2 < 75:
            color = "BLACK"
        elif 109 < average < 135:
            color = "BLUE"
        elif 135 < average < 165:
            color = "PURPLE"
        elif 166 < average < 175 and average2 < 170:
            color = "PINK"
        elif 170 < average:
            color = "RED"
        else:
            color = "PINK"

        return color
            

        
        