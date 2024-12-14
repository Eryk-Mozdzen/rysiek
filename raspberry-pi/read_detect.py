import cv2
import requests
import numpy as np
import time
from http.client import IncompleteRead  # Import for specific error handling

# ESP32-CAM stream URL
esp32_cam_url = "http://192.168.1.170/"

# Load Haar cascades for face detection
face_front_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
face_profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')

# Function to detect faces and annotate the frame
def detect_and_display_faces(frame):
    # Convert frame to grayscale (Haar cascades work better on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect front faces
    faces_front = face_front_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # Detect profile faces
    faces_profile = face_profile_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # Draw bounding boxes around detected front faces
    for (x, y, w, h) in faces_front:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if w > 150 or h > 150:  # Display "Hello" if the face is big enough
            cv2.putText(frame, "Hello", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Draw bounding boxes around detected profile faces
    for (x, y, w, h) in faces_profile:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        if w > 150 or h > 150:  # Display "Hello" if the face is big enough
            cv2.putText(frame, "Hello", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return frame

# Function to stream video from ESP32-CAM
def stream_esp32_cam(url):
    try:
        # Establish connection to the ESP32-CAM stream
        response = requests.get(url, stream=True, timeout=10)
        if response.status_code != 200:
            print("Failed to connect to ESP32-CAM stream. Check the URL.")
            return False
        
        byte_buffer = b""
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
                        # Perform face detection and annotate the frame
                        annotated_frame = detect_and_display_faces(image)

                        # Display the frame
                        cv2.imshow("ESP32-CAM Face Detection", annotated_frame)

                        # Press 'q' to exit the video stream
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            return True
    except (IncompleteRead, requests.exceptions.RequestException) as e:
        # Log error and trigger a restart
        print(f"Error: {e}. Restarting the stream in 1 seconds...")
        time.sleep(1)
        return False
    finally:
        cv2.destroyAllWindows()

# Main program entry point with retry mechanism
if __name__ == "__main__":
    print("Press 'q' to quit the application.")
    while True:
        success = stream_esp32_cam(esp32_cam_url)
        if success:
            break  # Exit the loop if the user presses 'q'