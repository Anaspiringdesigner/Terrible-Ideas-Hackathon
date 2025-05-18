import cv2
import mediapipe as mp
import math
import serial
import time
import keyboard

# Set up serial connection to Arduino
ser = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2) # Give some time to establish the connection

# Function to send the target motor angles to the Arduino
def send_motor_angle_to_arduino(x, y):
    # Format the message as "azimuthal_angle, longitudinal_angle\n"
    message = f"{x},{y}\n"
    
    # Send the message to the Arduino via serial
    ser.write(message.encode('utf-8'))
    print(f"Sent: x={x} degrees, y={y} degrees")

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh()
mp_drawing = mp.solutions.drawing_utils

# Function to calculate azimuthal (left-right) and longitudinal (up-down) angles
def calculate_angles(x, y, frame_width, frame_height):
    center_x = frame_width / 2
    center_y = frame_height / 2
    pixel_x = x * frame_width
    pixel_y = y * frame_height
    relative_x = pixel_x - center_x
    relative_y = pixel_y - center_y

    # Normalize the relative position to the range -1 to 1
    norm_x = relative_x / center_x
    norm_y = relative_y / center_y

    # Map to -45° to 45° range
    azimuthal_angle = norm_x * 45 # Left-right angle
    longitudinal_angle = -norm_y * 45 # Up-down angle (inverted y-axis)

    return azimuthal_angle, longitudinal_angle

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to RGB (MediaPipe expects RGB)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the frame and detect face landmarks
    results = face_mesh.process(frame_rgb)

    # Check if any face is detected
    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            nose_tip = face_landmarks.landmark[1] # Nose tip landmark
            h, w, _ = frame.shape

            # Calculate azimuthal (left-right) and longitudinal (up-down) angles
            azimuthal_angle, longitudinal_angle = calculate_angles(nose_tip.x, nose_tip.y, w, h)

            nose_x = int(nose_tip.x * w)
            nose_y = int(nose_tip.y * h)

            print(f"Nose detected at: X={nose_x}, Y={nose_y}")
            print(f"Azimuthal angle (left-right): {azimuthal_angle:.2f} degrees")
            print(f"Longitudinal angle (up-down): {longitudinal_angle:.2f} degrees")

            # Draw a circle at the nose tip on the frame
            cv2.circle(frame, (nose_x, nose_y), 5, (0, 255, 0), -1)

            # Combine azimuthal and longitudinal angles for display
            angle_text = f"Azimuthal: {azimuthal_angle:.2f}° | Longitudinal: {longitudinal_angle:.2f}°"
            cv2.putText(frame, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Send the calculated angles to Arduino
            send_motor_angle_to_arduino(azimuthal_angle, longitudinal_angle)

    else:
        print("No face detected.")

    # Display the frame with nose tracking
    cv2.imshow('Nose Tracking', frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if keyboard.is_pressed('q'): # If the 'q' key is pressed 
        # Release the webcam and close serial connection
        cap.release()
        ser.close()
        cv2.destroyAllWindows()
        break 

# Release the webcam and close serial connection
cap.release()
ser.close()
cv2.destroyAllWindows()