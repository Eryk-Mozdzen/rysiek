import cv2 as cv
import mediapipe as mp
import matplotlib.pyplot as plt
from collections import deque
import math
import time
import threading
import queue

from picocom import PicoCom

# Initialize MediaPipe Holistic and Drawing modules
mp_pose = mp.solutions.pose
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils


REAL_HEIGHT = 165
FOCAL_LENGTH = 250/REAL_HEIGHT


def main():
    # Initialize video capture
    com = PicoCom()
    cap = cv.VideoCapture(0)  # Change to video file path if needed
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv.CAP_PROP_FPS, 0.1)
    
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    plt.ion()

    frames_count = 0

    with mp_pose.Pose(
        min_detection_confidence=0.5, min_tracking_confidence=0.5
    ) as holistic:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture video")
                break

            frames_count += 1
            if frames_count % 5 != 0:
                continue

            fps = cvFpsCalc.get()
            # print(fps)
            frame = draw_text(frame, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255,255,255), 2, cv.LINE_AA)


            # Convert frame to RGB as MediaPipe requires RGB images
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            frame_rgb.flags.writeable = False

            # Perform holistic detection
            results = holistic.process(frame_rgb)

            frame_rgb.flags.writeable = True
            frame = cv.cvtColor(frame_rgb, cv.COLOR_RGB2BGR)

            
            
            # Check if pose landmarks are detected
            if results.pose_landmarks:
                
                mp_drawing.draw_landmarks(
                     frame, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS
                )
                draw_bounding_box(frame, results.pose_landmarks.landmark)
                # print(results.pose_landmarks.landmark[0])
                height, width, _ = frame.shape

                # for i in range(33):
                    # x = results.pose_landmarks.landmark[i].x * width
                    # y = results.pose_landmarks.landmark[i].y * height
                    # cv.putText(frame, f'{i}', (int(x), int(y)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv.LINE_AA)

                center_x = (
                    results.pose_landmarks.landmark[11].x * width
                    + results.pose_landmarks.landmark[12].x * width
                    + results.pose_landmarks.landmark[23].x * width
                    + results.pose_landmarks.landmark[24].x * width
                ) / 4
                center_y = (
                    results.pose_landmarks.landmark[11].y * height
                    + results.pose_landmarks.landmark[12].y * height
                    + results.pose_landmarks.landmark[23].y * height
                    + results.pose_landmarks.landmark[24].y * height
                ) / 4
                cv.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                radar = (center_x / width)
                angle = math.pi * radar
                
                radar_point = (width - 50, 50)
                
                cv.circle(frame, radar_point, 50, (0, 0, 0), -1)
                cv.circle(frame, radar_point, 50, (0, 255, 0), 2)
                
                cv.arrowedLine(frame, 
                               radar_point,
                               (radar_point[0] - int(40 * math.cos(angle)),
                                radar_point[1] - int(40 * math.sin(angle))),
                               (0, 255, 0), 2)
                
                draw_text(frame, f"{round(radar,2)}", (width-70, 130), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
                
                floor_pos = ((results.pose_landmarks.landmark[29].x + results.pose_landmarks.landmark[30].x) / 2, (results.pose_landmarks.landmark[29].y + results.pose_landmarks.landmark[30].y) / 2)
                nose_pos = (results.pose_landmarks.landmark[0].x, results.pose_landmarks.landmark[0].y)
                person_height = math.sqrt((floor_pos[0] - nose_pos[0]) ** 2 + (floor_pos[1] - nose_pos[1]) ** 2)
                distance = FOCAL_LENGTH * REAL_HEIGHT / person_height
                draw_text(frame, f"{int(distance)}", (width-70, 160), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv.LINE_AA)
                human_found = True
                human_radar = radar * 2 - 1
                
            else:
               human_found = False 
               human_radar = 0

            # if results.left_hand_landmarks:
            #     mp_drawing.draw_landmarks(
            #         frame, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS
            #     )
            #     draw_bounding_box(frame, results.left_hand_landmarks.landmark)
            # if results.right_hand_landmarks:
            #     mp_drawing.draw_landmarks(
            #         frame, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS
            #     )
            #     draw_bounding_box(frame, results.right_hand_landmarks.landmark)


            cv.imshow("Holistic Detection with Bounding Box", frame)

            if cv.waitKey(1) & 0xFF == ord("q"):
                break
            
            pwm = 150
            sleep = 0.15
            
            if human_found == False:
                com.send_cmd(r=255,g=255,b=0)
            else:
                if distance > 250:
                    com.send_cmd(g=255, rmot=pwm+30, lmot=pwm)
                    time.sleep(sleep*5)
                    com.send_cmd(g=255)

                elif abs(human_radar) < 0.2:
                    com.send_cmd(b=255)
                else:
                    if human_radar < 0:
                        com.send_cmd(g=255, rmot=pwm, lmot=-1 *pwm)
                    else:
                        com.send_cmd(g=255, rmot=-1 * pwm, lmot=pwm)
                    time.sleep(sleep)
                    com.send_cmd(g=255)

                
    
            # plt.show()
            # plt.pause(0.0001)

    # Release resources
    cap.release()
    cv.destroyAllWindows()


# Function to draw a bounding box
def draw_bounding_box(image, landmarks):
    height, width, _ = image.shape
    x_coords = [landmark.x * width for landmark in landmarks]
    y_coords = [landmark.y * height for landmark in landmarks]

    x_min, x_max = int(min(x_coords)), int(max(x_coords))
    y_min, y_max = int(min(y_coords)), int(max(y_coords))

    cv.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)


def draw_text(image, text, pos, font, scale, color, size, type):
    cv.putText(image, text, pos, font, scale, (0, 0, 0), size * 3, type)
    cv.putText(image, text, pos, font, scale, color, size, type)
    return image


class CvFpsCalc(object):
    def __init__(self, buffer_len=1):
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._difftimes = deque(maxlen=buffer_len)

    def get(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._difftimes.append(different_time)

        fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
        fps_rounded = round(fps, 2)

        return fps_rounded


if __name__ == "__main__":
    main()
