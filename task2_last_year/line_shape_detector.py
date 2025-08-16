import cv2
import numpy as np
import time
from task2_last_year.FrameCapture import Camera
from threading import Lock
from task2_last_year.SlicedImage import *
from task2_last_year.Functions import *
from threading import Thread
# from testfunctions import stabilize, autonomous, stayIdle, STOP, stableYaw, moveForward, moveBackward

pTime = 0
cTime = 0

frames_consistency = 25

def is_line_contour(contour):
    epsilon = 0.02 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    # Check if the polygon has 2 vertices (indicating a line)
    return len(approx) == 2

def nothing(x):
    pass

font = cv2.FONT_HERSHEY_COMPLEX
hull_image = None

sliced_images = []
N_SLICES = 15
for i in range(N_SLICES):
    sliced_images.append(SlicedImage()) #initialize the SlicedImage class

class LineShapeDetector:
    def __init__(self, videoCapturePort, images=sliced_images):
        self.cameraProcess = Camera(videoCapturePort)

        self.lower_white = np.array([110,110,110])
        self.upper_white = np.array([200, 200, 200])
        self.lower_red = np.array([0, 0, 140])
        self.upper_red = np.array([100, 100, 255])
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([80, 255, 60])
        self.slice_images = images
        self.num_slices = N_SLICES

        self.triangle_frames = []
        self.squre_frames = []
        self.rectangle_frames = []
        self.trombus_frames = []
        self.pentagon_frames = []
        self.hexagon_frames = []
        self.star_frames = []
        self.ellipse_frames = []
        self.yonca_frames = []
        self.circle_frames = []
        self.frame_counter = 0 # This is for chekcing the previous frames. If the detected shape appears consistent or not.

        self.num_triangles = 0
        self.num_rectangles = 0
        self.num_squares = 0
        self.num_trombus = 0
        self.num_pentagons = 0
        self.num_hexagons = 0
        self.num_star = 0
        self.num_ellipse = 0
        self.num_yonca = 0
        self.num_circle = 0

        self.shape_labels = ['Triangle', 'Rectangle', 'Pentagon', 'Hexagon', 'Star', 'Ellipse', 'Circle']
        self.captured_frames = {shape: 0 for shape in self.shape_labels} 
        self.detected_frames = {shape: 0 for shape in self.shape_labels}
        self.max_frames_per_shape = 5  # Set the desired number of frames per shape

        self.line_frame = None
        self.shape_frame = None

        self.running = False

        self.lock = Lock()


    def detect_line_and_shapes(self, frame):
        if self.slice_images is None:
            self.slice_images = sliced_images
        frameCopy = frame.copy()

        #-------- LINE DETECTION --------#
        # frameCopy = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([80, 255, 60])

        mask_color = cv2.inRange(frameCopy, lower_black, upper_black)

        img = frame.copy()
        img = cv2.bitwise_and(img, img, mask=mask_color)
        img = cv2.bitwise_not(img, img, mask=mask_color)
        img = (255 - img)  # Background of img is removed

        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        imgray = cv2.GaussianBlur(imgray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(imgray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        # Apply morphological operations to clean up the image
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)  # Closing: Fills small holes
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)   # Opening: Removes small noise

        # Find contours in the cleaned binary image
        contour_color, _ = cv2.findContours(mask_color.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contour_color, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        
        # Create a blank mask to draw and fill large contours
        mask_color = np.zeros_like(thresh)

        # Set minimum area to filter noise, adjust as needed
        min_area = 2000  
        for cnt in contour_color:
            if cv2.contourArea(cnt) > min_area:
                cv2.drawContours(mask_color, [cnt], -1, 255, thickness=cv2.FILLED)

        # Invert the mask to correct the colors
        line = cv2.bitwise_not(mask_color)

        SlicePart(line, sliced_images, N_SLICES)
        line = RepackImages(sliced_images)

        center_image = np.array(line.shape[1::-1]) // 2    
        center_contour = None
        min_distance = float('inf')
        for contour in contour_color:
            # Calculate the center of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # Calculate the Euclidean distance to the center of the image
                distance = np.linalg.norm(center_image - np.array([cX, cY]))
                # Check if this contour is closer than the previous minimum
                if distance < min_distance:
                    min_distance = distance
                    center_contour = contour
        # If a contour is found, draw a bounding box around it
        if center_contour is not None:
            x, y, w, h = cv2.boundingRect(center_contour)
            cv2.rectangle(line, (x, y), (x + w, y + h), (255, 0, 255), 2)

        max_contour = None

        # Threshold the image
        ret, threshold_img = cv2.threshold(mask_color, 127, 255, cv2.THRESH_BINARY)
        # Find contours in the threshold image
        contours, _ = cv2.findContours(threshold_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Draw the contours on the original image
        contours = [contour for contour in contours if cv2.matchShapes(max_contour, contour, cv2.CONTOURS_MATCH_I1, 0.0) > 0.01]
        cv2.drawContours(line, contours, -1, (0, 255, 0), 2)

        try:
            if contours:
                max_contour = max(contours, key = cv2.contourArea)
            # cv2.drawContours(frame, main_contour, -1, (255, 255, 255), 4)
        except Exception as e:
            print("exeption :", e)

        ### Debugging
        # if max_contour is not None and center_contour is not None:
        #     if max_contour.all() == center_contour.all():
        #         print("center_contour == max_contour")
        #     else:
        #         print("center_contour != max_contour")
        #     # max_contour = max_contour.astype(np.float32)
        #     if is_line_contour(max_contour):
        #         print("----- max_contour is line -----")

        self.slice_images = sliced_images
            
        #------ SHAPE DETECTION -------#  
        shape_frame = frame.copy()      
        hsv = cv2.cvtColor(shape_frame, cv2.COLOR_BGR2HSV)

        lower_limit = np.array([20, 100, 100])
        upper_limit = np.array([40, 255, 255])

        mask = cv2.inRange(hsv, lower_limit, upper_limit) # change the pixels which have color between upper red and lower red to white
        # cv2.imshow("Mask", mask)
        kernel = np.ones((5, 5), np.uint8)
        # cv2.imshow("kernel", kernel)
        opening = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # cv2.imshow("opening", opening)
        median = cv2.medianBlur(opening, 7)
        # cv2.imshow("median", median)

        if int(cv2.__version__[0]) > 3:
            contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 1000:
                cv2.drawContours(shape_frame, [approx], 0, (0, 0, 0), 2)

                if len(approx) == 3:
                    cv2.putText(shape_frame, "Triangle", (x, y), font, 1, (0,255,0))

                    self.triangle_frames.append(self.frame_counter)
                    last_five = self.triangle_frames[-frames_consistency:][::-1]
                    
                    is_consistent = False
                    is_consistent = self.check_consistency(last_five)
                    if self.num_triangles<5 and is_consistent:
                        filename = f"captured_Triangle_{self.num_triangles}.jpg"
                        cv2.imwrite(filename, shape_frame)
                        self.num_triangles += 1
                elif len(approx) == 10:
                    if self.num_triangles == 5:
                        name = "Star"
                    cv2.putText(shape_frame, name, (x, y), font, 1, (255,0,255))

                    self.star_frames.append(self.frame_counter)
                    last_five = self.star_frames[-frames_consistency:][::-1]
                    
                    is_consistent = False
                    is_consistent = self.check_consistency(last_five)
                    if self.num_star<5 and is_consistent and self.num_triangles == 5: # to avoid capturing star before triangle.(teknofest document)
                        filename = f"captured_Star_{self.num_star}.jpg"
                        cv2.imwrite(filename, shape_frame)
                        self.num_star += 1
                elif len(approx) == 4:
                    name = "Rectangle"
                    if self.num_star == 5 and self.num_ellipse < 3:
                        name = "square"
                        self.squre_frames.append(self.frame_counter)
                        last_five = self.squre_frames[-frames_consistency:][::-1]

                        is_consistent = False
                        is_consistent = self.check_consistency(last_five) 
                        if self.num_squares<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_squares}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_squares += 1
                    if self.num_ellipse == 5 and self.num_pentagons < 3: # not ==0 because of possible mistakes
                        name = "TROMBUS"
                        self.trombus_frames.append(self.frame_counter)
                        last_five = self.trombus_frames[-frames_consistency:][::-1]

                        is_consistent = False
                        is_consistent = self.check_consistency(last_five)
                        if self.num_trombus<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_trombus}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_trombus += 1
                    if self.num_hexagons == 5 and self.num_circle < 3:
                        name = "dikdortgen"
                        self.rectangle_frames.append(self.frame_counter)
                        last_five = self.rectangle_frames[-frames_consistency:][::-1]

                        is_consistent = False
                        is_consistent = self.check_consistency(last_five)
                        if self.num_rectangles<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_rectangles}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_rectangles += 1

                    cv2.putText(shape_frame, name, (x, y), font, 1, (255,0,0))

                elif 7 < len(approx) < 15: # 10 is star but is has been checked before already.
                    name = "eLiPsE"
                    if self.num_squares == 5 and self.num_trombus < 3:
                        name = "ellipse"
                        self.ellipse_frames.append(self.frame_counter)
                        last_five = self.ellipse_frames[-frames_consistency:][::-1]
                        is_consistent = False

                        is_consistent = self.check_consistency(last_five)
                        if self.num_ellipse<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_ellipse}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_ellipse += 1
                    if self.num_pentagons==5 and self.num_hexagons < 3:
                        name = "Yonca"
                        self.yonca_frames.append(self.frame_counter)
                        last_five = self.yonca_frames[-frames_consistency:][::-1]
                       
                        is_consistent = False
                        is_consistent = self.check_consistency(last_five)
                        if self.num_yonca<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_yonca}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_yonca += 1
                    if self.num_rectangles==5: 
                        name = "Circle"
                        self.circle_frames.append(self.frame_counter)
                        last_five = self.circle_frames[-frames_consistency:][::-1]

                        is_consistent = False
                        is_consistent = self.check_consistency(last_five)
                        if self.num_circle<5 and is_consistent:
                            filename = f"captured_{name}_{self.num_circle}.jpg"
                            cv2.imwrite(filename, shape_frame)
                            self.num_circle += 1
                    cv2.putText(shape_frame, name, (x, y), font, 1, (0,255,255))

                elif len(approx) == 5:
                    cv2.putText(shape_frame, "Pentagon", (x, y), font, 1, (0,0,255))
                    
                    self.pentagon_frames.append(self.frame_counter)
                    last_five = self.pentagon_frames[-frames_consistency:][::-1]
                    is_consistent = False

                    is_consistent = self.check_consistency(last_five)
                    if self.num_pentagons<5 and is_consistent and self.num_ellipse == 5: # to avoid capturing pentagon before ellipse.(teknofest document)
                        filename = f"captured_Pentagon_{self.num_pentagons}.jpg"
                        cv2.imwrite(filename, shape_frame)
                        self.num_pentagons += 1
                elif len(approx) == 6:
                    cv2.putText(shape_frame, "Hexagon", (x, y), font, 1, (0,255,255))

                    self.hexagon_frames.append(self.frame_counter)
                    last_five = self.hexagon_frames[-frames_consistency:][::-1]
                    is_consistent = False

                    is_consistent = self.check_consistency(last_five)
                    if self.num_hexagons<5 and is_consistent and self.num_pentagons == 5: # to avoid capturing hexagon before pentagon.(teknofest document)
                        filename = f"captured_Hexagon_{self.num_hexagons}.jpg"
                        cv2.imwrite(filename, shape_frame)
                        self.num_hexagons += 1

        self.line_frame = line
        self.shape_frame = shape_frame

        return shape_frame, line
    
    def run(self):
        self.running = True
        targetYaw = 0
        while self.running:
            if self.cameraProcess.isCameraOpened:
                try:
                    currentFrame = self.cameraProcess.getCurrentFrame()
                    if currentFrame is not None:
                        # count the frames, set a limit to avoid memory overflow ????
                        if self.frame_counter < 30000: # 15 minutes x 60 seconds x 33 fps
                            self.frame_counter += 1
                        else:
                            self.frame_counter = 0

                        frames = self.detect_line_and_shapes(currentFrame)
                        shape = frames[0]
                        line = frames[1]

                        cv2.imshow("Shape", shape)
                        cv2.imshow("Line", line)

                        ## autonomous part to test on local ##

                        print("auto.vehicle.Channel5 = 1600") # go forward slowly to find a line
                        line_slices = self.slice_images
                        
                        front_slices = line_slices[:5] # Total is lineShape_detector.num_slices == 15
                        # 0-4 front, 5-9 middle, 10-14 back

                        slice_counter = 0 # counts how many of first slices 
                        for slice in front_slices: # check first 5 slices. 
                            if slice.includes_line:
                                slice_counter += 1
                        
                        if slice_counter >= 3: # if more than 3 slices include line (black contour)
                            folow_line = True
                        else:
                            folow_line = False

                        startOfLine = -1
                        for slice in front_slices: 
                            startOfLine += 1
                            if slice.includes_line:
                                break # find the first part of the line
                            else:
                                startOfLine = -1

                        if startOfLine != -1: # if there is no line in front
                            head_line_position_0 = line_slices[startOfLine].error # is 100 if at right most of the frame
                            head_line_position_1 = line_slices[startOfLine+1].error
                            head_line_position_2 = line_slices[startOfLine+2].error
                        else:
                            head_line_position_0 = 0

                        middle = int(self.num_slices/2) - 1 # should be 6 for 15
                        error_middle_1 = line_slices[middle].error - line_slices[middle+2].error 
                        error_middle_2 = line_slices[middle+1].error - line_slices[middle+3].error
                        error_middle_3 = line_slices[middle+2].error - line_slices[middle+4].error

                        
                        if (error_middle_1>30 and error_middle_2>30) or (error_middle_2>30 and error_middle_3>30): 
                            new_angle = targetYaw+90
                            # stableYaw(target_yaw=new_angle)
                            print("turn right")
                        elif error_middle_1<-30 and error_middle_2<-30 or error_middle_2<-30 and error_middle_3<-30:
                            new_angle = targetYaw-90
                            # stableYaw(target_yaw=new_angle)          
                            print("turn left")
                        # maybe should create a thread for this: 
                        elif abs(head_line_position_0)>15 and abs(head_line_position_2)>15: # trying to follow line by correcting the yaw
                            error_angel = head_line_position_0/4
                            targetYaw = error_angel
                            print("targstabetYaw: ", targetYaw)


                        line_slices = self.slice_images

                    key = cv2.waitKey(150)
                    if key == ord('q'):
                        self.running = False
                        break
                    # time.sleep(0.03)
                except Exception as e:
                    print(e)
                    # time.sleep(3)

        self.cameraProcess.isActive = False
        self.cameraProcess.cameraThread.join()
        cv2.destroyAllWindows()

    def get_sliceImages(self):
        return self.slice_images
    
    def check_consistency(self, frame_list):
        i = 0
        for frame in frame_list:
            if frame == self.frame_counter - i:
                is_consistent = True
            else:
                is_consistent = False
            i += 1
        return is_consistent
 
if __name__ == "__main__":
    
    gstreamer_pipeline_str = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw, width=640, height=480, framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
    )
        
    detector = LineShapeDetector("/Users/mac/Desktop/abis desktop/Xavier/kayit_2025-08-02_14-37-41.mp4")
    line_shape_thread = Thread(target=detector.run, args=())
    # line_shape_thread.start()
    detector.run()
