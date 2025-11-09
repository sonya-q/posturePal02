import cv2
import mediapipe as mp
import numpy as np
import time
import subprocess
import platform
import serial
import threading

arduino = serial.Serial('/dev/tty.usbmodemXXXX', 9600)  # Adjust the port as necessary

def play_sound():
    subprocess.run(["afplay", "dog_bark.mp3"])

def send_signal(status):
    """Send posture status to Arduino."""
    if status == "GOOD":
        arduino.write(b'GOOD')  # Send good posture signal to Arduino
    elif status == "WARN":
        arduino.write(b'WARN')  # Send warning signal to Arduino
    elif status == "BAD":
        arduino.write(b'BAD')    # Send bad posture signal to Arduino

bark_played = False
bad_posture_start = None
good_posture_start = None
arduino_posture_state = "GOOD"  # Last posture sent to Arduino
BAD_POSTURE_DELAY = 3          # seconds
GOOD_POSTURE_DELAY = 5          # seconds

# Hardcoded Arduino timing (independent of posture detection)
arduino_start_time = None
ARDUINO_GOOD_DURATION = 5.0   # 0-5 seconds: send GOOD
ARDUINO_WARN_DURATION = 10.0  # 5-10 seconds: send WARN
# After 10 seconds: send BAD

# Initialize calibration variables
is_calibrated = False
calibration_frames = 0
calibration_shoulder_angles = []
calibration_neck_angles = []
calibration_forward_distances = []
calibration_slouch_distances = []
calibration_spine_curvatures = []
shoulder_threshold = 0
neck_threshold = 0
forward_head_threshold = 0
slouch_threshold = 0
spine_curvature_threshold = 0
CALIBRATION_TARGET = 60


def calculate_angle(a, b, c):
    """Calculate the angle between three points."""
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    
    ba = a - b
    bc = c - b
    
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    angle = np.degrees(angle)
    
    return angle

def calculate_distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def calculate_spine_curvature(points):
    """Calculate spine curvature by fitting a polynomial curve."""
    if len(points) < 3:
        return 0
    
    x_coords = np.array([p[0] for p in points])
    y_coords = np.array([p[1] for p in points])
    
    coeffs = np.polyfit(y_coords, x_coords, 2)
    curvature = abs(coeffs[0]) * 10000
    
    return curvature

def draw_angle(frame, point1, point2, point3, angle, color=(0, 255, 0)):
    """Draw the angle on the frame with lines and text."""
    cv2.line(frame, point1, point2, color, 2)
    cv2.line(frame, point2, point3, color, 2)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (point2[0] - 50, point2[1] - 10)
    cv2.putText(frame, f"{angle:.1f}°", org, font, 0.7, color, 2)

def send_macos_notification(title, message, subtitle=""):
    """Send a notification on macOS using osascript."""
    if platform.system() == "Darwin":
        script = f'''
        display notification "{message}" with title "{title}" subtitle "{subtitle}" sound name "Glass"
        '''
        try:
            subprocess.run(["osascript", "-e", script], check=True)
        except Exception as e:
            print(f"Notification error: {e}")
    else:
        print(f"{title}: {message}")

# Initialize MediaPipe Pose and webcam
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

cap = cv2.VideoCapture(0)

# Initialize alert variables
last_alert_time = 0
alert_cooldown = 30.0

print("Starting posture monitor with improved side-view detection...")
print("Please maintain GOOD POSTURE for calibration:")
print("  • Sit up straight")
print("  • Head back (ears over shoulders)")
print("  • Chest up, shoulders back")
print("\nArduino signals: GOOD (0-5s) → WARN (5-10s) → BAD (10s+)")
posture_start_time = time.time()


while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(image)
    
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        
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
        nose = (int(landmarks[mp_pose.PoseLandmark.NOSE].x * w),
                int(landmarks[mp_pose.PoseLandmark.NOSE].y * h))
        left_hip = (int(landmarks[mp_pose.PoseLandmark.LEFT_HIP].x * w),
                   int(landmarks[mp_pose.PoseLandmark.LEFT_HIP].y * h))
        right_hip = (int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP].x * w),
                    int(landmarks[mp_pose.PoseLandmark.RIGHT_HIP].y * h))
        left_elbow = (int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x * w),
                     int(landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y * h))
        right_elbow = (int(landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x * w),
                      int(landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y * h))
        left_knee = (int(landmarks[mp_pose.PoseLandmark.LEFT_KNEE].x * w),
                    int(landmarks[mp_pose.PoseLandmark.LEFT_KNEE].y * h))
        right_knee = (int(landmarks[mp_pose.PoseLandmark.RIGHT_KNEE].x * w),
                     int(landmarks[mp_pose.PoseLandmark.RIGHT_KNEE].y * h))

        # Calculate midpoints
        shoulder_midpoint = ((left_shoulder[0] + right_shoulder[0])//2,
                            (left_shoulder[1] + right_shoulder[1])//2)
        ear_midpoint = ((left_ear[0] + right_ear[0])//2,
                       (left_ear[1] + right_ear[1])//2)
        hip_midpoint = ((left_hip[0] + right_hip[0])//2,
                       (left_hip[1] + right_hip[1])//2)
        elbow_midpoint = ((left_elbow[0] + right_elbow[0])//2,
                         (left_elbow[1] + right_elbow[1])//2)
        knee_midpoint = ((left_knee[0] + right_knee[0])//2,
                        (left_knee[1] + right_knee[1])//2)
        
        # Create spine points
        upper_back = ((shoulder_midpoint[0] + elbow_midpoint[0])//2,
                     (shoulder_midpoint[1] + elbow_midpoint[1])//2)
        lower_back = ((elbow_midpoint[0] + hip_midpoint[0])//2,
                     (elbow_midpoint[1] + hip_midpoint[1])//2)
        
        spine_points = [
            ear_midpoint,
            shoulder_midpoint,
            upper_back,
            lower_back,
            hip_midpoint
        ]

        # Calculate measurements
        shoulder_angle = calculate_angle(left_shoulder, right_shoulder, (right_shoulder[0], 0))
        neck_angle = calculate_angle(ear_midpoint, shoulder_midpoint, (shoulder_midpoint[0], 0))
        forward_head_distance = abs(ear_midpoint[0] - shoulder_midpoint[0])
        slouch_distance = abs(shoulder_midpoint[0] - hip_midpoint[0])
        spine_curvature = calculate_spine_curvature(spine_points)
        
        # Draw visual indicators
        cv2.line(frame, ear_midpoint, (shoulder_midpoint[0], ear_midpoint[1]), (0, 255, 255), 2)
        cv2.line(frame, shoulder_midpoint, (hip_midpoint[0], shoulder_midpoint[1]), (255, 0, 255), 2)
        
        # Draw spine curve
        y_coords = np.array([p[1] for p in spine_points])
        x_coords = np.array([p[0] for p in spine_points])
        
        if len(spine_points) >= 3:
            coeffs = np.polyfit(y_coords, x_coords, 2)
            y_smooth = np.linspace(y_coords.min(), y_coords.max(), 50)
            x_smooth = np.polyval(coeffs, y_smooth)
            
            for i in range(len(x_smooth) - 1):
                pt1 = (int(x_smooth[i]), int(y_smooth[i]))
                pt2 = (int(x_smooth[i+1]), int(y_smooth[i+1]))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 3)
        
        for point in spine_points:
            cv2.circle(frame, point, 5, (0, 255, 0), -1)
        
        cv2.circle(frame, ear_midpoint, 7, (0, 255, 255), -1)
        cv2.circle(frame, shoulder_midpoint, 7, (255, 255, 0), -1)
        cv2.circle(frame, hip_midpoint, 7, (255, 0, 255), -1)
        
        # Calibration phase
        if not is_calibrated and calibration_frames < CALIBRATION_TARGET:
            calibration_shoulder_angles.append(shoulder_angle)
            calibration_neck_angles.append(neck_angle)
            calibration_forward_distances.append(forward_head_distance)
            calibration_slouch_distances.append(slouch_distance)
            calibration_spine_curvatures.append(spine_curvature)
            calibration_frames += 1
            
            progress_text = f"Calibrating... {calibration_frames}/{CALIBRATION_TARGET}"
            instruction_text = "Good posture: Head back, sit tall!"
            
            cv2.putText(frame, progress_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, instruction_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            cv2.putText(frame, f"Shoulder angle: {shoulder_angle:.1f}°", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Neck angle: {neck_angle:.1f}°", (10, 135),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Forward head: {forward_head_distance:.1f}px", (10, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Slouch: {slouch_distance:.1f}px", (10, 185),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Spine curve: {spine_curvature:.2f}", (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            if calibration_frames == CALIBRATION_TARGET:
                shoulder_threshold = np.mean(calibration_shoulder_angles) * 0.75
                neck_threshold = np.mean(calibration_neck_angles) * 0.75
                forward_head_threshold = np.mean(calibration_forward_distances) * 1.8
                slouch_threshold = np.mean(calibration_slouch_distances) * 1.5
                spine_curvature_threshold = np.mean(calibration_spine_curvatures) * 1.5
                
                is_calibrated = True
                # Start Arduino timer after calibration
                arduino_start_time = time.time()
                
                print(f"\n✓ Calibration complete!")
                print(f"  Shoulder angle threshold: {shoulder_threshold:.1f}°")
                print(f"  Neck angle threshold: {neck_threshold:.1f}°")
                print(f"  Forward head threshold: {forward_head_threshold:.1f}px")
                print(f"  Slouch threshold: {slouch_threshold:.1f}px")
                print(f"  Spine curvature threshold: {spine_curvature_threshold:.2f}")
        
        # Posture monitoring phase
        elif is_calibrated:
            current_time = time.time()
            posture_issues = []
            detailed_corrections = []

            # Check angles (lower is bad)
            if shoulder_angle < shoulder_threshold:
                posture_issues.append("shoulders")
                detailed_corrections.append("SHOULDERS: Pull back and down, open chest")

            if neck_angle < neck_threshold:
                posture_issues.append("neck tilt")
                detailed_corrections.append("NECK: Keep head level, don't tilt")

            # Check distances (higher is bad)
            if forward_head_distance > forward_head_threshold:
                posture_issues.append("forward head")
                detailed_corrections.append("HEAD: Too far forward - tuck chin, align ears over shoulders")

            if slouch_distance > slouch_threshold:
                posture_issues.append("slouching")
                detailed_corrections.append("SPINE: Slouching forward - sit up tall, engage core")

            # Check spine curvature (higher = bad)
            if spine_curvature > spine_curvature_threshold:
                posture_issues.append("spine curvature")
                detailed_corrections.append("BACK: Spine too curved - straighten upper back, open chest")

            # Track bad posture for audio and notifications (unchanged)
            if posture_issues:
                if bad_posture_start is None:
                    bad_posture_start = current_time
                elif current_time - bad_posture_start >= BAD_POSTURE_DELAY and not bark_played:
                    threading.Thread(target=play_sound).start()
                    bark_played = True
            else:
                bad_posture_start = None
                bark_played = False

            # HARDCODED ARDUINO SIGNALS - Based on time elapsed, not posture
            arduino_elapsed = current_time - arduino_start_time

            send_signal("GOOD")
            time(3)
            send_signal("WARN")
            time(7)
            send_signal("BAD") 
            time(10) 
            send_signal("WARN")
            time(5)
            send_signal("BAD") 
            time(10)
            send_signal("WARN")
            time(5)
            send_signal("GOOD")
            time(10)
            send_signal("GOOD")
            time(3)
            send_signal("WARN")
            time(7)
            send_signal("BAD") 
            time(10) 
            send_signal("WARN")
            time(5)
            send_signal("BAD") 
            time(10)
            send_signal("WARN")
            time(5)
            send_signal("GOOD")
            time(10)
            send_signal("GOOD")
            time(3)
            send_signal("WARN")
            time(7)
            send_signal("BAD") 
            time(10) 
            send_signal("WARN")
            time(5)
            send_signal("BAD") 
            time(10)
            send_signal("WARN")
            time(5)
            send_signal("GOOD")
            time(10)
            # if arduino_elapsed < ARDUINO_GOOD_DURATION:
            #     # 0-5 seconds: Send GOOD
            #     send_signal("GOOD")
            #     arduino_status = f"Arduino: GOOD ({ARDUINO_GOOD_DURATION - arduino_elapsed:.1f}s left)"
            # elif arduino_elapsed < ARDUINO_WARN_DURATION:
            #     # 5-10 seconds: Send WARN
            #     send_signal("WARN")
            #     arduino_status = f"Arduino: WARN ({ARDUINO_WARN_DURATION - arduino_elapsed:.1f}s left)"
            # else:
            #     # 10+ seconds: Send BAD
            #     send_signal("BAD")
            #     arduino_status = f"Arduino: BAD ({arduino_elapsed - ARDUINO_WARN_DURATION:.1f}s)"

            # Display status based on actual posture detection (for CV display only)
            if posture_issues:
                status = "Poor Posture ⚠"
                color = (0, 0, 255)  # Red

                # Send macOS notification only if cooldown has passed
                if current_time - last_alert_time > alert_cooldown:
                    print(f"\n⚠️  POSTURE CORRECTION NEEDED:")
                    for correction in detailed_corrections:
                        print(f"   • {correction}")
                    print()

                    issue_summary = ", ".join(posture_issues)
                    main_correction = detailed_corrections[0] if detailed_corrections else "Check your posture"
                    send_macos_notification(
                        title="⚠️ Posture Alert",
                        message=main_correction,
                        subtitle=f"Issues: {issue_summary}"
                    )
                    last_alert_time = current_time
            else:
                status = "Good Posture ✓"
                color = (0, 255, 0)  # Green

            # Display status and measurements
            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.putText(frame, f"S:{shoulder_angle:.0f}° N:{neck_angle:.0f}° "
                                f"FH:{forward_head_distance:.0f}px Sl:{slouch_distance:.0f}px "
                                f"SC:{spine_curvature:.2f}",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
            
            # Display Arduino signal status
            cv2.putText(frame, arduino_status, (10, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)

    # Display the frame
    cv2.imshow('Posture Monitor - Fixed Side View', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()