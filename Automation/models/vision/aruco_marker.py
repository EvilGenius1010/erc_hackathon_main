import cv2
import cv2.aruco as aruco

# Load 5x5 ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)

# Create Detector Parameters
parameters = aruco.DetectorParameters()

# Create ArUco Detector Object
detector = aruco.ArucoDetector(aruco_dict, parameters)

# Open default webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        print("Detected ArUco IDs:", ids.flatten())

    # Display result
    cv2.imshow("ArUco Marker Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
