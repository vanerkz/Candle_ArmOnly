import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_id = 0
marker_size = 50

# generate marker (grayscale)
img = aruco.drawMarker(aruco_dict, marker_id, marker_size)

# convert grayscale -> RGB
img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# save RGB image
cv2.imwrite("aruco_0.png", img_rgb)