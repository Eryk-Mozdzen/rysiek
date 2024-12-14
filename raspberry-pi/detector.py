import cv2

# Load Haar cascade for face detection
face_front_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
face_profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_profileface.xml')

# Open a connection to the laptop camera (0 = default camera)
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Accessing laptop camera...")

while True:
    # Capture frame-by-frame from the camera
    ret, frame = camera.read()
    
    if not ret:
        print("Error: Failed to read frame from camera.")
        break
    
    # Convert frame to grayscale (Haar cascades work better on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the frame
    faces_front = face_front_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    # faces_profile = face_profile_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    faces_profile = face_profile_cascade.detectMultiScale(gray)
    
    # Draw bounding boxes around detected faces_front
    for (x, y, w, h) in faces_front:
        # Draw rectangle around the detected face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Check if the detected face is "big enough" (arbitrary threshold, e.g., width > 150 pixels)
        if w > 150 or h > 150:
            # Display the "Hello" message on the frame
            cv2.putText(frame, "Hello", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    for (x, y, w, h) in faces_profile:
        # Draw rectangle around the detected face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Check if the detected face is "big enough" (arbitrary threshold, e.g., width > 150 pixels)
        if w > 150 or h > 150:
            # Display the "Hello" message on the frame
            cv2.putText(frame, "Hello", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Display the frame in a window
    cv2.imshow('Face Detection', frame)
    
    # Press 'q' to exit the video display
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the display window
camera.release()
cv2.destroyAllWindows()
