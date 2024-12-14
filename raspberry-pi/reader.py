import cv2
import requests
import numpy as np

url = "http://192.168.1.170/"

try:
    response = requests.get(url, stream=True)
    if response.status_code != 200:
        print("Failed to connect to ESP32-CAM stream. Check the URL.")

    byte_buffer = b""
    for chunk in response.iter_content(chunk_size=1024):
        byte_buffer += chunk
        start = byte_buffer.find(b'\xff\xd8')
        end = byte_buffer.find(b'\xff\xd9')

        if start != -1 and end != -1:
            jpg_data = byte_buffer[start:end + 2]
            byte_buffer = byte_buffer[end + 2:]

            image = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)

            if image is not None:
                cv2.imshow("ESP32-CAM Stream", image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

except Exception as e:
    print(e)

finally:
    cv2.destroyAllWindows()
