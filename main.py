import cv2
import mediapipe as mp
import numpy as np
import time

def calculate_angle(a, b, c):
    """
    Calculate the angle between three points.
    Args:
        a: First point (x,y)
        b: Mid point (x,y)
        c: End point (x,y)
    Returns:
        angle: The angle in degrees
    """
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    
    # Calculate vectors
    ba = a - b
    bc = c - b
    
    # Calculate angle using dot product
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    
    # Convert to degrees
    angle = np.degrees(angle)
    
    return angle

def draw_angle(frame, point1, point2, point3, angle, color=(0, 255, 0)):
    """
    Draw the angle on the frame with lines and text.
    """
    cv2.line(frame, point1, point2, color, 2)
    cv2.line(frame, point2, point3, color, 2)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (point2[0] - 50, point2[1] - 10)
    cv2.putText(frame, f"{angle:.1f}°", org, font, 0.7, color, 2)

# Initialize MediaPipe Pose and webcam
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
cap = cv2.VideoCapture(0)

# Initialize calibration variables
is_calibrated = False
calibration_frames = 0
calibration_shoulder_angles = []
calibration_neck_angles = []
calibration_chin_neck_angles = []  # NEW
calibration_spine_angles = []      # NEW
shoulder_threshold = 0
neck_threshold = 0
chin_neck_threshold = 0  # NEW
spine_threshold = 0      # NEW
CALIBRATION_TARGET = 60

# Initialize alert variables
last_alert_time = 0
alert_cooldown = 5.0

print("Starting posture monitor with side-view detection...")
print("Please maintain GOOD POSTURE (sit up straight, head back) for the first 60 frames!")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert BGR to RGB
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the image and detect pose
    results = pose.process(image)
    
    if results.pose_landmarks:
        # Draw pose landmarks
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        
        # Get landmarks
        landmarks = results.pose_landmarks.landmark
        h, w = frame.shape[:2]
        
        # Extract key points
        left_shoulder = (int(landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x * w),
                        int(landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y * h))
        right_shoulder = (int(landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x * w),
                         int(landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y * h))
        left_ear = (int(landmarks[mp_pose.PoseLandmark.LEFT_EAR].x * w),
                   int(landmarks[mp_pose.PoseLandmark.LEFT_EAR].y * h))
        right_ear = (int(landmarks[mp_pose.PoseLandmark.RIGHT_EAR].x * w),
                    int(landmarks[mp_pose.PoseLandmark.RIGHT_EAR].y * h))
        
        # NEW: Additional landmarks for side-view posture
        nose = (int(landmarks[mp_pose.PoseLandmark.NOSE].x * w),
                int(landmarks[mp_pose.PoseLandmark.NOSE].y * h))
        left_hip = (int(landmarks[mp_pose.PoseLandmark.LEFT_HIP].x * w),
                   int(landmarks[mp_pose.PoseLandmark.LEFT_HIP].y * h))
        right_hip = (int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x * w),
                    int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y * h))

        # Calculate existing angles
        shoulder_angle = calculate_angle(left_shoulder, right_shoulder, (right_shoulder[0], 0))
        neck_midpoint = ((left_ear[0] + right_ear[0])//2, (left_ear[1] + right_ear[1])//2)
        neck_bottom = ((left_shoulder[0] + right_shoulder[0])//2, (left_shoulder[1] + right_shoulder[1])//2)
        neck_angle = calculate_angle(neck_midpoint, neck_bottom, (neck_bottom[0], 0))

        # NEW: Calculate chin-to-neck angle (forward head posture)
        # Angle between nose, neck midpoint, and shoulder midpoint
        chin_neck_angle = calculate_angle(nose, neck_midpoint, neck_bottom)
        
        # NEW: Calculate spine angle (upper back slouch)
        # Angle between shoulder midpoint, hip midpoint, and vertical reference
        hip_midpoint = ((left_hip[0] + right_hip[0])//2, (left_hip[1] + right_hip[1])//2)
        spine_angle = calculate_angle(neck_bottom, hip_midpoint, (hip_midpoint[0], hip_midpoint[1] + 100))

        # Calibration phase
        if not is_calibrated and calibration_frames < CALIBRATION_TARGET:
            calibration_shoulder_angles.append(shoulder_angle)
            calibration_neck_angles.append(neck_angle)
            calibration_chin_neck_angles.append(chin_neck_angle)  # NEW
            calibration_spine_angles.append(spine_angle)          # NEW
            calibration_frames += 1
            
            # Display calibration progress
            progress_text = f"Calibrating... {calibration_frames}/{CALIBRATION_TARGET}"
            instruction_text = "Sit STRAIGHT: Head back, chest up!"
            
            cv2.putText(frame, progress_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, instruction_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Show current angles
            cv2.putText(frame, f"Shoulder: {shoulder_angle:.1f}°", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Neck: {neck_angle:.1f}°", (10, 135),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Chin-Neck: {chin_neck_angle:.1f}°", (10, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Spine: {spine_angle:.1f}°", (10, 185),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if calibration_frames == CALIBRATION_TARGET:
                shoulder_threshold = np.mean(calibration_shoulder_angles) * 0.9
                neck_threshold = np.mean(calibration_neck_angles) * 0.9
                chin_neck_threshold = np.mean(calibration_chin_neck_angles) * 0.85  # NEW: Stricter for forward head
                spine_threshold = np.mean(calibration_spine_angles) * 0.85         # NEW: Stricter for slouching
                is_calibrated = True
                print(f"\nCalibration complete!")
                print(f"Shoulder threshold: {shoulder_threshold:.1f}°")
                print(f"Neck threshold: {neck_threshold:.1f}°")
                print(f"Chin-Neck threshold: {chin_neck_threshold:.1f}°")
                print(f"Spine threshold: {spine_threshold:.1f}°")
        
        # Posture monitoring phase
        elif is_calibrated:
            current_time = time.time()
            posture_issues = []
            
            # Check all four angles
            if shoulder_angle < shoulder_threshold:
                posture_issues.append("shoulders")
            if neck_angle < neck_threshold:
                posture_issues.append("neck tilt")
            if chin_neck_angle < chin_neck_threshold:
                posture_issues.append("forward head")
            if spine_angle < spine_threshold:
                posture_issues.append("spine slouch")
            
            if posture_issues:
                status = "Poor Posture"
                color = (0, 0, 255)  # Red
                issue_text = f"Issues: {', '.join(posture_issues)}"
                
                if current_time - last_alert_time > alert_cooldown:
                    print(f"Poor posture detected! {issue_text}")
                    last_alert_time = current_time
                
                cv2.putText(frame, issue_text, (10, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                status = "Good Posture"
                color = (0, 255, 0)  # Green
            
            # Draw angles
            draw_angle(frame, left_shoulder, right_shoulder, (right_shoulder[0], 0), shoulder_angle)
            draw_angle(frame, neck_midpoint, neck_bottom, (neck_bottom[0], 0), neck_angle, color=(255, 0, 0))
            draw_angle(frame, nose, neck_midpoint, neck_bottom, chin_neck_angle, color=(0, 255, 255))  # Cyan
            draw_angle(frame, neck_bottom, hip_midpoint, (hip_midpoint[0], hip_midpoint[1] + 100), spine_angle, color=(255, 0, 255))  # Magenta
            
            # Display status
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.putText(frame, f"Angles: S:{shoulder_angle:.0f} N:{neck_angle:.0f} CN:{chin_neck_angle:.0f} Sp:{spine_angle:.0f}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Display the frame
    cv2.imshow('Posture Monitor - Side View Enhanced', frame)
    
    # Break the loop on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
