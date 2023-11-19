import cv2
import numpy as np

class Camera:
    def __init__(self):
        # make sure that the webcam can see the whole checkerboard by
        # running webcam(2).preview in the Command Window
        self.cam = cv2.VideoCapture(2) # Get camera object
        self.params = self.calibrate() # Run Calibration Function
        self.cam_IS, self.cam_pose = self.calculateCameraPos()

    def getTForm(self):
        return self.cam_TForm

    def getCameraPose(self):
        return self.cam_pose

    def getCameraInstrinsics(self):
        return self.cam_IS

    def getRotationMatrix(self):
        return self.cam_R

    def getTranslationVector(self):
        return self.cam_T

    def shutdown(self):
        # shutdown script which clears camera variable
        self.cam.release()

    def calibrate(self):
        # Calibration function
        # This function will run the camera calibration, save the camera parameters,
        # and check to make sure calibration worked as expected
        # The calibrate function will ask if you are ready. To calibrate, you must press
        # any key, then the system will confirm if the calibration is successful
        # NOTE: This uses the camcalib.m file for camera calibration. If you have placed
        # your camera calibration script elsewhere, you will need to change the command below
        params = 0
        try:
            print("Clear surface of any items, then press any key to continue")
            input()
            print("Calibrating")
            # Change this if you are using a different calibration script
            # camcalib()
            params = cameraParams
            print("Camera calibration complete!")
        except Exception as e:
            print(str(e))
            print("No camera calibration file found. Plese run camera calibration")
        return params

    # Returns an undistorted camera image
    def getImage(self):
        ret, raw_img = self.cam.read()
        img, new_origin = cv2.undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full')
        return img

    def calculateCameraPos(self):  # DO NOT USE
        # Get transformation from camera to checkerboard frame
        # This function will get the camera position based on checkerboard.
        # You should run this function every time the camera position is changed.
        # It will calculate the extrinsics, and output to a transformation matrix.
        # Keep in mind: this transformation matrix is a transformation from pixels
        # to x-y coordinates in the checkerboard frame!
        # 1. Capture image from camera
        ret, raw_img = self.cam.read()
        # 2. Undistort Image based on params
        img, newIs = cv2.undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full')
        # 3. Detect checkerboard in the image
        imagePoints, boardSize = cv2.detectCheckerboardPoints(img, 'PartialDetections', False)
        # 4. Compute transformation
        self.params.WorldPoints = self.params.WorldPoints[self.params.WorldPoints[:, 1] <= (boardSize[1]-1)*25, :]
        worldPointSize = self.params.WorldPoints.shape
        imagePointSize = imagePoints.shape
        print("World Points is {} x {}".format(worldPointSize[0], worldPointSize[1]))
        print("Image Points is {} x {}".format(imagePointSize[0], imagePointSize[1]))
        print("The checkerboard is {} squares long x {} squares wide".format(boardSize[0], boardSize[1]))
        # 4. Compute transformation
        R, t = cv2.extrinsics(imagePoints, self.params.WorldPoints, newIs)
        self.cam_R = R
        self.cam_T = t
        self.cam_TForm = np.array([[R[0, 0], R[0, 1], R[0, 2], t[0]],
                                   [R[1, 0], R[1, 1], R[1, 2], t[1]],
                                   [R[2, 0], R[2, 1], R[2, 2], t[2]],
                                   [0, 0, 0, 1]])
        pose = np.array([[R[0, 0], R[0, 1], R[0, 2], t[0]],
                         [R[1, 0], R[1, 1], R[1, 2], t[1]],
                         [R[2, 0], R[2, 1], R[2, 2], t[2]],
                         [0, 0, 0, 1]])
        return newIs, pose


