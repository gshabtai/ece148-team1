import json
import cv2 as cv
import numpy as np

class JSONManager():
    def __init__(self):
        f = open('settings.json')
        self.settings = json.load(f)
        f.close

        # Set global parameters
        self.calibration_mode = self.settings['calibration_mode']

        # Set Color detection paramenters
        self.lower_hue1 = self.settings['color_detection']['lower_hue1']
        self.lower_sat1 = self.settings['color_detection']['lower_sat1']
        self.lower_val1 = self.settings['color_detection']['lower_val1']
        self.upper_hue1 = self.settings['color_detection']['upper_hue1']
        self.upper_sat1 = self.settings['color_detection']['upper_sat1']
        self.upper_val1 = self.settings['color_detection']['upper_val1']

    def save_settings(self):
        if self.calibration_mode:
            # Save color detection parameters
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
    # Helper functions
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

        if self.calibration_mode:
            # Set passed parameters
            self.windowName = windowName
            # Create a named window
            cv.namedWindow(self.windowName)
            cv.namedWindow('Mask')
            # Create trackbars
            cv.createTrackbar('LowerHue1', 'Mask', self.lower_hue1, 254, self.update_lower_hue1)
            cv.createTrackbar('LowerSat1', 'Mask', self.lower_sat1, 254, self.update_lower_sat1)
            cv.createTrackbar('LowerVal1', 'Mask', self.lower_val1, 254, self.update_lower_val1)
            cv.createTrackbar('UpperHue1', 'Mask', self.upper_hue1, 254, self.update_upper_hue1)
            cv.createTrackbar('UpperSat1', 'Mask', self.upper_sat1, 254, self.update_upper_sat1)
            cv.createTrackbar('UpperVal1', 'Mask', self.upper_val1, 254, self.update_upper_val1)

        print("Done Loading")
    
    def update_image(self):
        ''' Update image from video source and find centroid of circle'''

        # Capture new image from source
        _, frame = self.cap.read()
        frame = cv.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
        
        self.hsv_search(frame)

        if self.calibration_mode:
            cv.imshow(self.windowName, frame)

    def moment_search(self,frame, mask):
        '''calculate moments of binary image'''
        M = cv.moments(mask)

        if int(M["m00"]) != 0:
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # put text and highlight the center
            cv.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
            cv.putText(frame, "centroid", (cX - 25, cY - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    def hsv_search(self, frame):
        # convert to hsv colorspace
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array([self.lower_hue1,self.lower_sat1,self.lower_val1]) , 
            np.array([self.upper_hue1, self.upper_sat1, self.upper_val1]))
        self.moment_search(frame,mask)

        cv.imshow('Mask', mask)

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