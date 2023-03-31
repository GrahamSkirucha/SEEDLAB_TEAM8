import cv2
import numpy as np

# Load the ArUco dictionary and parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

# Capture the image
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# Detect the ArUco marker
corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

# Estimate the pose of the marker
if ids is not None:
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
   
    # Calculate the distance in meters
    distance = np.sqrt(tvecs[0][0]**2 + tvecs[0][1]**2 + tvecs[0][2]**2) / 1000.0
    print("Distance to ArUco marker:", distance, "meters")