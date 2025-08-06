import cv2
import numpy as np

# Load the image
frame = cv2.imread("test_image_blue.jpeg")  # Make sure this image is in the same folder
if frame is None:
    print("Image not found!")
    exit()

# ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
parameters = cv2.aruco.DetectorParameters()

# Define HSV color ranges
color_ranges = {
    'Red': [(0, 100, 100), (10, 255, 255)],
    'Green': [(45, 100, 100), (75, 255, 255)],
    'Blue': [(100, 100, 100), (130, 255, 255)],
    'Yellow': [(20, 100, 100), (35, 255, 255)]
}

# Convert to HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

detected_color = "None"
max_pixels = 0
best_mask = None

# Color detection and mask creation
for color, (lower, upper) in color_ranges.items():
    lower = np.array(lower)
    upper = np.array(upper)
    mask = cv2.inRange(hsv, lower, upper)
    pixel_count = cv2.countNonZero(mask)

    if pixel_count > max_pixels:
        max_pixels = pixel_count
        detected_color = color
        best_mask = mask

print(f"Detected color: {detected_color}")

# Draw bounding box around largest color blob
if best_mask is not None:
    contours, _ = cv2.findContours(best_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:  # Avoid small noise
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)

# Convert to grayscale and detect ArUco markers
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

if ids is not None:
    for i in range(len(ids)):
        cv2.aruco.drawDetectedMarkers(frame, corners)
        cv2.putText(frame, f"ID: {ids[i][0]}", (10, 70 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

# Overlay detected color label
cv2.putText(frame, f"Color: {detected_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

# Show the result
cv2.imshow('Warehouse Detection', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


