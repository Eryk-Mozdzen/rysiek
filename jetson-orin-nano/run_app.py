#!/usr/bin/env python
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import csv
import copy
import itertools
from collections import deque
import cv2 as cv
import numpy as np
import mediapipe as mp
import tensorflow as tf


def main():

    # Camera ###############################################################
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 540)
    cvFpsCalc = CvFpsCalc(buffer_len=10)

    # Model #############################################################
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        max_num_hands=2,
    )
    gesture_classifier = GestureClassifier()
    
    conf_threshold = 0.5

    # Labels ###########################################################
    with open('data/gesture_labels.csv',
              encoding='utf-8-sig') as f:
        gesture_classifier_labels = csv.reader(f)
        gesture_classifier_labels = [
            row[1] for row in gesture_classifier_labels
        ]

    #  ########################################################################
    mode = 0
    number = -1
    
    while True:
        fps = cvFpsCalc.get()
        
        # Process Key (ESC: end) #################################################
        record = False
        key = cv.waitKey(10)
        # print(key)
        if key == 27:  # ESC
            break
        elif 48 <= key <= 57:  # 0 ~ 9
            number = key - 48
        elif key == 13:  # enter
            mode = 1 - mode
        elif key == 32:  # space
            record = True

        # Camera capture #####################################################
        ret, image = cap.read()
        if not ret:
            break
        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)

        # Detection implementation #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        #  ####################################################################
        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                # Bounding box calculation
                brect = calc_bounding_rect(debug_image, hand_landmarks)
                # Landmark calculation
                landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                # Normalizing coordiantes
                pre_processed_landmark_list = pre_process_landmark(
                    landmark_list)

                # Write to the dataset file
                if mode == 1 and record:
                    record_sample(number, pre_processed_landmark_list)

                # Hand gesture classification
                hand_gesture_id, confidence = gesture_classifier(pre_processed_landmark_list)
                # print(confidence)
                # Drawing
                debug_image = draw_bounding_rect(debug_image, brect)
                debug_image = draw_landmarks(debug_image, hand_landmarks)
                
                debug_image = draw_info_text(
                    debug_image,
                    brect,
                    handedness,
                    gesture_classifier_labels[hand_gesture_id] if confidence > conf_threshold else "",
                    confidence
                )

        debug_image = draw_info(debug_image, fps, mode, number, record, gesture_classifier_labels)

        # Display #############################################################
        cv.imshow('Hand Gesture Recognition', debug_image)

    cap.release()
    cv.destroyAllWindows()


def select_mode(key, mode, number):
    if 48 <= key <= 57:  # 0 ~ 9
        number = key - 48
    if key == 110:  # n
        mode = 0
    if key == 107:  # k
        mode = 1
    return number, mode


def calc_bounding_rect(image, landmarks):
    offset = 20
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        
        landmark_point = [np.array((landmark_x, landmark_y))]
        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x-offset, y-offset, x + w + offset, y + h + offset]


def calc_landmark_list(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []

    # Keypoint
    for _, landmark in enumerate(landmarks.landmark):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point.append([landmark_x, landmark_y])

    return landmark_point


def pre_process_landmark(landmark_list):
    temp_landmark_list = copy.deepcopy(landmark_list)

    # Convert to relative coordinates
    base_x, base_y = 0, 0
    for index, landmark_point in enumerate(temp_landmark_list):
        if index == 0:
            base_x, base_y = landmark_point[0], landmark_point[1]

        temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
        temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

    # Convert to a one-dimensional list
    temp_landmark_list = list(
        itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


def record_sample(number, landmark_list):
    if (0 <= number <= 9):
        csv_path = f'data/gestures/{number}.csv'
        
        with open(csv_path, 'a', newline="") as f:
            writer = csv.writer(f)
            writer.writerow([number, *landmark_list])
    return


def draw_landmarks(image, hand_landmarks):
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    
    mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp.solutions.hands.HAND_CONNECTIONS,
            mp_drawing_styles.DrawingSpec(color=[255, 255, 255], thickness=2, circle_radius=3),
            mp_drawing_styles.DrawingSpec(color=[255, 255, 255], thickness=2, circle_radius=3),
        )
    return image

def draw_bounding_rect(image, brect):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0,0,0), 2)
    return image


def draw_info_text(image, brect, handedness, hand_sign_text, confidence):
    cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

    info_text = handedness.classification[0].label[0:]
    
    if hand_sign_text != "":
        info_text = info_text + ':' + hand_sign_text + f' ({int(confidence*100)}%)'
    cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
    return image

def draw_text(image, text, pos, font, scale, color, size, type):
    cv.putText(image, text, pos, font,
               scale, (0,0,0), size*3, type)
    cv.putText(image, text, pos, font,
               scale, color, size, type)
    return image


def draw_info(image, fps, mode, number, record, labels):
    image = draw_text(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
               1.0, (255,255,255), 2, cv.LINE_AA)

    image = draw_text(image, "ESC - exit", (500, 30),
        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
        cv.LINE_AA)
    
    
    image = draw_text(image, "ENTER - start/stop recording", (500, 50),
        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
        cv.LINE_AA)


    if 1 <= mode:
        rec_txt = f"{number} - {labels[number] if len(labels) > number else 'unknown'}" if number > -1 else '-'
        
        image = draw_text(image, f"Learning new gesture ({rec_txt})", (10, 90),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                   cv.LINE_AA)
        if record:
            image = draw_text(image, f"*RECORDING* ({sum(1 for _ in open(f'data/gestures/{number}.csv'))})", (10, 110),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                       cv.LINE_AA)
            
        image = draw_text(image, "SPACE - record", (500, 70),
            cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
            cv.LINE_AA)
        
        image = draw_text(image, "[0-9] - select dataset", (500, 90),
            cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
            cv.LINE_AA)
    
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
    

class GestureClassifier(object):
    def __init__(
        self,
        model_path='model/gesture_classifier.tflite',
        num_threads=16,
    ):
        self.interpreter = tf.lite.Interpreter(model_path=model_path,
                                               num_threads=num_threads)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def __call__(
        self,
        landmark_list,
    ):
        input_details_tensor_index = self.input_details[0]['index']
        self.interpreter.set_tensor(
            input_details_tensor_index,
            np.array([landmark_list], dtype=np.float32))
        self.interpreter.invoke()

        output_details_tensor_index = self.output_details[0]['index']

        result = self.interpreter.get_tensor(output_details_tensor_index)

        result_index = np.argmax(np.squeeze(result))

        return result_index, np.squeeze(result)[result_index]
    


if __name__ == '__main__':
    main()
