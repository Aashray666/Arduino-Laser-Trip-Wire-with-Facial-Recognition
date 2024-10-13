import serial
import time
import face_recognition
import cv2
import os
import numpy as np
from datetime import datetime

# Load known faces
def load_known_faces(known_faces_dir):
    known_face_encodings = []
    known_face_names = []

    print(f"Loading known faces from: {known_faces_dir}")
    for file_name in os.listdir(known_faces_dir):
        if file_name.endswith(".jpg") or file_name.endswith(".png"):
            image = face_recognition.load_image_file(f"{known_faces_dir}/{file_name}")
            encoding = face_recognition.face_encodings(image)[0]
            known_face_encodings.append(encoding)
            known_face_names.append(os.path.splitext(file_name)[0])
            print(f"Loaded {file_name} as {os.path.splitext(file_name)[0]}")

    print(f"Total known faces loaded: {len(known_face_names)}")
    return known_face_encodings, known_face_names

# Capture video from webcam and perform face recognition
def run_face_recognition(ser, known_face_encodings, known_face_names, max_captures=5, capture_delay=5):
    print("Running face recognition...")

    # Capture video from webcam
    video_capture = cv2.VideoCapture(0)

    if not video_capture.isOpened():
        print("Error: Could not open webcam.")
        return

    capture_count = 0  # Initialize counter for captured images

    while capture_count < max_captures:
        ret, frame = video_capture.read()

        if not ret:
            print("Error: Failed to capture video frame.")
            break

        # Convert the frame from BGR (OpenCV) to RGB (face_recognition)
        rgb_frame = frame[:, :, ::-1]

        # Find all faces in the current frame of video
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        for face_encoding in face_encodings:
            # Compare with known faces
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)

            # Save the image regardless of whether the face is known or unknown
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_name = f"Captured_Face_{timestamp}.jpg"
            cv2.imwrite(img_name, frame)
            print(f"Captured and saved image: {img_name}")
            capture_count += 1

            # Check if the face matches any known faces
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
                print(f"Matched with {name}")

                # Send signal to Arduino to stop the buzzer if face is known
                ser.write(b'STOP_BUZZER\n')
                print("Sent signal to stop the buzzer.")
            else:
                print("No match found. Saving as unknown.")

            if capture_count >= max_captures:
                print(f"Reached the maximum of {max_captures} images. Stopping capture.")
                break

        # Display the video feed
        cv2.imshow('Video', frame)

        # Introduce a delay between captures (time in seconds)
        print(f"Waiting for {capture_delay} seconds before the next capture...")
        time.sleep(capture_delay)

        # Quit with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video capture and close all windows
    video_capture.release()
    cv2.destroyAllWindows()

# Listen to Arduino
def listen_to_arduino(serial_port, known_face_encodings, known_face_names, max_captures=5, capture_delay=5):
    ser = serial.Serial(serial_port, 9600, timeout=1)
    time.sleep(2)  # Wait for serial connection to establish
    print("Listening for laser trip signal...")

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Received: {line}")

            if line == "TRIP":  # Assuming Arduino sends "TRIP" when laser is tripped
                print("Laser trip detected! Running face recognition.")
                run_face_recognition(ser, known_face_encodings, known_face_names, max_captures, capture_delay)
                break  # Exit after taking the limited number of images
            else:
                print("Laser not tripped yet.")

if __name__ == "__main__":
    known_faces_dir = "known_faces"  # Ensure this folder exists
    known_face_encodings, known_face_names = load_known_faces(known_faces_dir)

    # Replace 'COM3' with your Arduino's serial port (e.g., /dev/ttyUSB0 on Linux/Mac or COMx on Windows)
    serial_port = 'COM3'
    listen_to_arduino(serial_port, known_face_encodings, known_face_names, max_captures=5, capture_delay=5)
