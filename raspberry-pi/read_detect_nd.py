import cv2
import requests
import numpy as np
import time
import serial
from http.client import IncompleteRead  # Import for specific error handling

# ESP32-CAM stream URL
esp32_cam_url = "http://192.168.1.170/"

# Load Haar cascades for face detection
face_front_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
face_profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')

ser = serial.Serial(port='/dev/serial0', baudrate=19200, timeout=1) 

def send_uart_message(message):
    """Sends a message over the UART serial connection."""
    try:
        ser.write((message).encode())
    except Exception as e:
        print(f"UART Error: {e}")

# Function to detect faces and annotate the frame
def detect_and_log_faces(frame, frame_count):
    # Get the frame's width to classify face position
    frame_width = frame.shape[1]  # Width of the frame
    
    # Convert frame to grayscale (Haar cascades work better on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect front faces
    faces_front = face_front_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # Detect profile faces
    faces_profile = face_profile_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    detected_faces = False  # Flag to check if any face is detected

    # Log and display detected front faces
    for (x, y, w, h) in faces_front:
        detected_faces = True
        char = '0'
        print(f"[FRAME {frame_count}] Front face detected at (x={x}, y={y}, w={w}, h={h})")
        
        # Draw a rectangle around the face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle for front faces

        # Check position of the face
        face_center_x = x + w // 2  # Get the center of the face
        if face_center_x < frame_width * (1/3):
            position = "Left"
            char = 'L'
        elif face_center_x < frame_width * (2/3):
            position = "Middle"
            char = 'C'
        else:
            position = "Right"
            char = 'R'
        
        print(f"[FRAME {frame_count}] Front face position: {position}")
        send_uart_message(char)
        
        # Display the position on the frame
        cv2.putText(frame, position, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # Green text

        if w > 150 or h > 150:
            print(f"[FRAME {frame_count}] Large front face detected. (Displaying Hello)")

    # Log and display detected profile faces
    for (x, y, w, h) in faces_profile:
        detected_faces = True
        print(f"[FRAME {frame_count}] Profile face detected at (x={x}, y={y}, w={w}, h={h})")
        
        # Draw a rectangle around the face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue rectangle for profile faces

        # Check position of the face
        face_center_x = x + w // 2  # Get the center of the face
        if face_center_x < frame_width * (1/3):
            position = "Left"
            char = 'L'
        elif face_center_x < frame_width * (2/3):
            position = "Middle"
            char = 'C'
        else:
            position = "Right"
            char = 'R'
        
        print(f"[FRAME {frame_count}] Profile face position: {position}")
        send_uart_message(char)
        
        # Display the position on the frame
        cv2.putText(frame, position, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)  # Blue text

        if w > 150 or h > 150:
            print(f"[FRAME {frame_count}] Large profile face detected. (Displaying Hello)")

    # If no faces are detected, print 0
    if not detected_faces:
        print(f"[FRAME {frame_count}] 0")
        char = '0'
        send_uart_message(char)

# Function to stream video from ESP32-CAM
def stream_esp32_cam(url):
    try:
        # Establish connection to the ESP32-CAM stream
        response = requests.get(url, stream=True, timeout=10)
        if response.status_code != 200:
            print("Failed to connect to ESP32-CAM stream. Check the URL.")
            return False
        
        byte_buffer = b""
        frame_count = 0
        print("Streaming from ESP32-CAM...")
        while True:
            # Read chunks of the stream
            for chunk in response.iter_content(chunk_size=1024):
                byte_buffer += chunk
                start = byte_buffer.find(b'\xff\xd8')  # Start of JPEG
                end = byte_buffer.find(b'\xff\xd9')    # End of JPEG

                if start != -1 and end != -1:
                    # Extract JPEG image data
                    jpg_data = byte_buffer[start:end + 2]
                    byte_buffer = byte_buffer[end + 2:]

                    # Decode the JPEG image
                    image = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)

                    if image is not None:
                        frame_count += 1
                        # Perform face detection and log the results
                        detect_and_log_faces(image, frame_count)

                        # If you want to save detected frames to disk, uncomment below:
                        # cv2.imwrite(f"frame_{frame_count}.jpg", image)

                        # Exit the loop if the script is running for too long or certain condition met (e.g., max frames)
                        # For now, we exit manually by pressing `Ctrl+C` in the terminal
    except (IncompleteRead, requests.exceptions.RequestException) as e:
        # Log error and trigger a restart
        print(f"Error: {e}. Restarting the stream in 1 second...")
        time.sleep(1)
        return False
    finally:
        print("Cleaning up resources...")

# Main program entry point with retry mechanism
if __name__ == "__main__":
    print("Starting headless face detection. Press 'Ctrl+C' to quit.")
    while True:
        success = stream_esp32_cam(esp32_cam_url)
        if success:
            break  # Exit the loop if the stream completes successfully
