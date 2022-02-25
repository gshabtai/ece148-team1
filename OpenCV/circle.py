from itertools import count
import json
import cv2 as cv
import numpy as np
from time import sleep

class JSONManager():
    def __init__(self):
        f = open('settings.json')
        self.settings = json.load(f)
        f.close

        # Set Circle detection parameters
        self.kernel = (self.settings['circle_detection']['blur_kernel_size'], self.settings['circle_detection']['blur_kernel_size'])
        self.calibration_mode = self.settings['calibration_mode']
        self.dp = self.settings['circle_detection']['dp']
        self.minDist = self.settings['circle_detection']['minDist']
        self.param1 = self.settings['circle_detection']['param1']
        self.param2 = self.settings['circle_detection']['param2']
        self.minRadius = self.settings['circle_detection']['minRadius']
        self.maxRadius = self.settings['circle_detection']['maxRadius']

        # Set Color detection paramenters
        self.lower_hue1 = self.settings['color_detection']['lower_hue1']
        self.lower_sat1 = self.settings['color_detection']['lower_sat1']
        self.lower_val1 = self.settings['color_detection']['lower_val1']
        self.upper_hue1 = self.settings['color_detection']['upper_hue1']
        self.upper_sat1 = self.settings['color_detection']['upper_sat1']
        self.upper_val1 = self.settings['color_detection']['upper_val1']

    def save_settings(self):
        if self.calibration_mode:
            # Update values before saving
            self.settings['circle_detection']['dp'] = self.dp
            self.settings['circle_detection']['minDist'] = self.minDist
            self.settings['circle_detection']['param1'] = self.param1
            self.settings['circle_detection']['param2'] = self.param2
            self.settings['circle_detection']['minRadius'] = self.minRadius
            self.settings['circle_detection']['maxRadius'] = self.maxRadius

            self.settings['color_detection']['lower_hue1'] = self.lower_hue1
            self.settings['color_detection']['lower_sat1'] = self.lower_sat1
            self.settings['color_detection']['lower_val1'] = self.lower_val1
            self.settings['color_detection']['upper_hue1'] = self.upper_hue1
            self.settings['color_detection']['upper_sat1'] = self.upper_sat1
            self.settings['color_detection']['upper_val1'] = self.upper_val1

            f = open('settings.json','w')
            f.write(json.dumps(self.settings, indent=4))
            f.close()

class DetectCircle(JSONManager):
    # Private class members
    prevCircle = None

    # Lambda functions
    dist = lambda _, x,y: (x[0]-x[1])**2+(y[0]-y[1])**2
    def update_dp(self, value): self.dp = (value+100)/100
    def update_minDist(self, value): self.minDist = value
    def update_param1(self, value): self.param1 = value
    def update_param2(self, value): self.param2 = value
    def update_minRadius(self, value):
        if self.maxRadius < self.minRadius:
            self.maxRadius = self.minRadius + 1
            cv.setTrackbarPos('maxRadius', self.windowName,self.maxRadius)
        self.minRadius = value
    def update_maxRadius(self, value): 
        if self.maxRadius < self.minRadius:
            self.minRadius = self.maxRadius - 1
            cv.setTrackbarPos('minRadius', self.windowName,self.minRadius)
        self.maxRadius = value
    def update_lower_hue1(self, value): self.lower_hue1 = value
    def update_lower_sat1(self, value): self.lower_sat1 = value
    def update_lower_val1(self, value): self.lower_val1 = value
    def update_upper_hue1(self, value): self.upper_hue1 = value
    def update_upper_sat1(self, value): self.upper_sat1 = value
    def update_upper_val1(self, value): self.upper_val1 = value
    

    def __init__(self, windowName='Webcam'):
        ''' This method is encharged of correctly initiallizing the detect Circle objects'''
        
        print("Starting... Please wait...")
        # Inherit parent properties
        super().__init__()

        # Sequester the video capture device
        self.cap = cv.VideoCapture(0)

        # Check if the webcam is opened correctly
        if not self.cap.isOpened():
            raise IOError("Cannot open webcam")

        print("Done Loading Core")
        if self.calibration_mode:
            # Wait
            print("Loading GUI, please wait")
            
            # Set passed parameters
            self.windowName = windowName
            # Create a named window
            cv.namedWindow(self.windowName)
            cv.namedWindow('Mask')
            # Create trackbars
            # cv.createTrackbar('maxRadius', self.windowName, int(self.maxRadius), 1000, self.update_maxRadius)
            # cv.createTrackbar('minRadius', self.windowName, int(self.minRadius), 1000, self.update_minRadius)
            # cv.createTrackbar('param2', self.windowName, int(self.param2), 1000, self.update_param2)
            # cv.createTrackbar('param1', self.windowName, int(self.param1), 1000, self.update_param1)
            # cv.createTrackbar('minDist', self.windowName, int(self.minDist), 1000, self.update_minDist)
            # cv.createTrackbar('dp', self.windowName, int(self.dp*100)-100, 100, self.update_dp)
            cv.createTrackbar('LowerHue1', 'Mask', self.lower_hue1, 254, self.update_lower_hue1)
            cv.createTrackbar('LowerSat1', 'Mask', self.lower_sat1, 254, self.update_lower_sat1)
            cv.createTrackbar('LowerVal1', 'Mask', self.lower_val1, 254, self.update_lower_val1)
            cv.createTrackbar('UpperHue1', 'Mask', self.upper_hue1, 254, self.update_upper_hue1)
            cv.createTrackbar('UpperSat1', 'Mask', self.upper_sat1, 254, self.update_upper_sat1)
            cv.createTrackbar('UpperVal1', 'Mask', self.upper_val1, 254, self.update_upper_val1)
    
    def update_image(self):
        ''' Update image from video source and find centroid of circle'''

        # Capture new image from source
        _, frame = self.cap.read()
        frame = cv.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)

        
        mask = self.hsv_search(frame)

        # calculate moments of binary image
        M = cv.moments(mask)

        if int(M["m00"]) != 0:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # put text and highlight the center
            cv.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv.putText(frame, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        if self.calibration_mode:
            cv.imshow(self.windowName, frame)

    def hsv_search(self, frame):
        # convert to hsv colorspace
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array([self.lower_hue1,self.lower_sat1,self.lower_val1]) , 
            np.array([self.upper_hue1, self.upper_sat1, self.upper_val1]))
        # mask2 = cv.inRange(hsv, np.array([160,0,0]) , np.array([180, 255, 255]))

        # mask = mask1 | mask2

        cv.imshow('Mask', mask)

        return mask
            
    def circle_search(self, frame):
        # Get rid of color becuase not needed
        # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Remove high frequency conponents from image
        blur = cv.GaussianBlur(frame, self.kernel, 0)

        # Vote circles on Hough parameter space
        circles = cv.HoughCircles(image=blur,method=cv.HOUGH_GRADIENT,
            dp=self.dp,minDist=self.minDist, param1=self.param1, 
            param2=self.param2, minRadius=self.minRadius, 
            maxRadius=self.maxRadius)

        # Select local maxima in an accumulator matrix.
        if circles is not None:
            circles = np.uint16(np.around(circles))
            chosen = None
            for i in circles[0,:]:
                if chosen is None: chosen = i
                if self.prevCircle is not None:
                    if self.dist(chosen, self.prevCircle) <= self.dist(i, self.prevCircle):
                        chosen = i
            cv.circle(frame,(chosen[0],chosen[1]), 1, (0,100,100),3)
            cv.circle(frame,(chosen[0],chosen[1]), chosen[2], (255,0,100),3)
            self.prevCircle = chosen
            
        if self.calibration_mode: 
            font = cv.FONT_HERSHEY_SIMPLEX
            cv.putText(frame,f'dp: {self.dp}',(25, 25),font,0.5,(255,255,255))
            cv.putText(frame,f'minDist: {self.minDist}',(25, 50),font,0.5,(255,255,255))
            cv.putText(frame,f'param1: {self.param1}',(25, 75),font,0.5,(255,255,255))
            cv.putText(frame,f'param2: {self.param2}',(25, 100),font,0.5,(255,255,255))
            cv.putText(frame,f'minRadius: {self.minRadius}',(25, 125),font,0.5,(255,255,255))
            cv.putText(frame,f'maxRadius: {self.maxRadius}',(25, 150),font,0.5,(255,255,255))

def main():
    detector = DetectCircle()
    
    while True:
        detector.update_image()
        # Wait for q keypress or KeyboardInterrupt event to occur
        if cv.waitKey(1) & 0xFF == ord('q'):
            detector.save_settings() # Choose to save settings if in calibration mode
            break

    
    

if __name__ == '__main__':
    main()