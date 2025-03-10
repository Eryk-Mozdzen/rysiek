import cv2 as cv
import mediapipe as mp
import numpy as np
import threading
import math
import queue
import json
import time
import signal
from dataclasses import dataclass
from typing import Optional, Tuple, List
from nucleocom import NucleoCom
import tensorflow as tf
import csv
import itertools
import copy


# Konfiguracja systemu
class Config:
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    FRAME_SKIP = 2  # Co którą klatkę przetwarzamy
    PROCESSING_THREADS = 2  # Liczba wątków obliczeniowych
    REAL_HEIGHT = 165
    FOCAL_LENGTH = 250 / REAL_HEIGHT
    ENABLE_PREVIEW = True  # Flaga włączająca/wyłączająca podgląd


@dataclass
class Frame:
    index: int
    frame: Optional[cv.Mat]
    processed: bool = False
    human_found: bool = False
    distance: float = 0
    human_radar: float = 0
    gesture: Optional[str] = None
    pose_results: Optional[object] = None  # Zapisujemy wyniki detekcji do wizualizacji


class FrameBuffer:
    def __init__(self, size: int):
        self.frames = [None] * size  # Bufor na określoną liczbę zdjęć
        self.current_index = 0
        self.lock = threading.Lock()
        self.size = size

    def put(self, frame: Frame) -> None:
        with self.lock:
            self.frames[self.current_index] = frame
            self.current_index = (self.current_index + 1) % self.size

    def get_unprocessed(self) -> Optional[Frame]:
        with self.lock:
            for frame in self.frames:
                if frame and not frame.processed:
                    return frame
        return None


class GestureClassifier(object):
    def __init__(
        self,
        model_path="model/gesture_classifier.tflite",
        num_threads=16,
    ):
        self.interpreter = tf.lite.Interpreter(
            model_path=model_path, num_threads=num_threads
        )

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def __call__(
        self,
        landmark_list,
    ):
        input_details_tensor_index = self.input_details[0]["index"]
        self.interpreter.set_tensor(
            input_details_tensor_index, np.array([landmark_list], dtype=np.float32)
        )
        self.interpreter.invoke()

        output_details_tensor_index = self.output_details[0]["index"]

        result = self.interpreter.get_tensor(output_details_tensor_index)

        result_index = np.argmax(np.squeeze(result))

        return result_index, np.squeeze(result)[result_index]


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
    temp_landmark_list = list(itertools.chain.from_iterable(temp_landmark_list))

    # Normalization
    max_value = max(list(map(abs, temp_landmark_list)))

    def normalize_(n):
        return n / max_value

    temp_landmark_list = list(map(normalize_, temp_landmark_list))

    return temp_landmark_list


class VisionSystem:
    def __init__(
        self,
        num_threads=Config.PROCESSING_THREADS,
        enable_preview=Config.ENABLE_PREVIEW,
    ):
        self.num_threads = num_threads
        self.enable_preview = enable_preview
        self.frame_buffer = FrameBuffer(size=num_threads)
        self.result_queue = queue.Queue()
        self.preview_queue = queue.Queue(maxsize=1)  # Kolejka do wyświetlania
        self.processing_threads: List[threading.Thread] = []
        self.stop_event = threading.Event()
        self.thread_ready = [threading.Event() for _ in range(num_threads)]

        # Konfiguracja obsługi sygnałów
        signal.signal(signal.SIGINT, self.handle_interrupt)
        signal.signal(signal.SIGTERM, self.handle_interrupt)

        # Inicjalizacja MediaPipe
        self.mp_pose = mp.solutions.pose
        self.mp_holistic = mp.solutions.holistic
        self.mp_drawing = mp.solutions.drawing_utils

        self.gesture_classifier = GestureClassifier()

        with open("data/gesture_labels.csv", encoding="utf-8-sig") as f:
            self.gesture_classifier_labels = csv.reader(f)
            self.gesture_classifier_labels = [
                row[1] for row in self.gesture_classifier_labels
            ]

        self.stop_mode = True
        self.gesture_counter = 0

    def handle_interrupt(self, signum, frame):
        print("\nZatrzymywanie programu...")
        self.stop_event.set()
        for event in self.thread_ready:
            event.set()
        cv.destroyAllWindows()

    def draw_visualization(self, frame_data: Frame) -> None:
        frame = frame_data.frame.copy()
        height, width, _ = frame.shape
        results = frame_data.pose_results

        if frame_data.pose_results and frame_data.pose_results.pose_landmarks:
            # Rysowanie punktów charakterystycznych i połączeń
            self.mp_drawing.draw_landmarks(
                frame,
                frame_data.pose_results.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
            )

            # Rysowanie boxa
            landmarks = frame_data.pose_results.pose_landmarks.landmark
            x_coords = [landmark.x * width for landmark in landmarks]
            y_coords = [landmark.y * height for landmark in landmarks]
            x_min, x_max = int(min(x_coords)), int(max(x_coords))
            y_min, y_max = int(min(y_coords)), int(max(y_coords))
            cv.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Rysowanie radaru
            center_x = (
                results.pose_landmarks.landmark[11].x * width
                + results.pose_landmarks.landmark[12].x * width
                + results.pose_landmarks.landmark[23].x * width
                + results.pose_landmarks.landmark[24].x * width
            ) / 4
            radar = center_x / width
            radar_point = (width - 50, 50)
            angle = math.pi * radar

            cv.circle(frame, radar_point, 50, (0, 0, 0), -1)
            cv.circle(frame, radar_point, 50, (0, 255, 0), 2)

            cv.arrowedLine(
                frame,
                radar_point,
                (
                    radar_point[0] - int(40 * math.cos(angle)),
                    radar_point[1] - int(40 * math.sin(angle)),
                ),
                (0, 255, 0),
                2,
            )

        # Rysowanie informacji zawsze
        status = "Detected" if frame_data.human_found else "Not detected"
        cv.putText(
            frame,
            f"Status: {status}",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )
        cv.putText(
            frame,
            f"Distance: {round(frame_data.distance, 2)}",
            (10, 60),
            cv.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )
        cv.putText(
            frame,
            f"Radar: {round(frame_data.human_radar, 2)}",
            (10, 90),
            cv.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )

        # Aktualizacja podglądu
        try:
            self.preview_queue.put_nowait(frame)
        except queue.Full:
            try:
                self.preview_queue.get_nowait()
                self.preview_queue.put_nowait(frame)
            except (queue.Empty, queue.Full):
                pass

    def preview_thread(self):
        while not self.stop_event.is_set():
            try:
                frame = self.preview_queue.get(timeout=1)
                cv.imshow("Vision System Preview", frame)
                if cv.waitKey(1) & 0xFF == ord("q"):
                    self.handle_interrupt(None, None)
            except queue.Empty:
                continue

    def frame_generator(self):
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, Config.CAMERA_WIDTH)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, Config.CAMERA_HEIGHT)

        frame_count = 0

        try:
            while not self.stop_event.is_set() and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Błąd kamery - próba ponownego połączenia...")
                    cap.release()
                    time.sleep(1)
                    cap = cv.VideoCapture(0)
                    continue

                frame_count += 1
                if frame_count % Config.FRAME_SKIP == 0:
                    frame_data = Frame(index=frame_count, frame=frame)
                    self.frame_buffer.put(frame_data)

                    # Powiadom wątki o nowych danych
                    for event in self.thread_ready:
                        event.set()
        finally:
            cap.release()

    def process_frame(self, thread_id: int):
        with self.mp_holistic.Holistic(
            min_detection_confidence=0.5, min_tracking_confidence=0.5
        ) as pose:
            while not self.stop_event.is_set():
                self.thread_ready[thread_id].wait()
                self.thread_ready[thread_id].clear()

                if self.stop_event.is_set():
                    break

                frame_data = self.frame_buffer.get_unprocessed()
                if frame_data is None:
                    continue
                frame_data.processed = True

                frame_rgb = cv.cvtColor(frame_data.frame, cv.COLOR_BGR2RGB)
                frame_rgb.flags.writeable = False
                results = pose.process(frame_rgb)
                frame_data.pose_results = results

                if results.pose_landmarks:
                    height, width, _ = frame_data.frame.shape
                    center_x = (
                        sum(
                            results.pose_landmarks.landmark[i].x
                            for i in [11, 12, 23, 24]
                        )
                        / 4
                    )
                    radar = center_x

                    floor_pos = (
                        (
                            results.pose_landmarks.landmark[29].x
                            + results.pose_landmarks.landmark[30].x
                        )
                        / 2,
                        (
                            results.pose_landmarks.landmark[29].y
                            + results.pose_landmarks.landmark[30].y
                        )
                        / 2,
                    )
                    nose_pos = (
                        results.pose_landmarks.landmark[0].x,
                        results.pose_landmarks.landmark[0].y,
                    )
                    person_height = (
                        (floor_pos[0] - nose_pos[0]) ** 2
                        + (floor_pos[1] - nose_pos[1]) ** 2
                    ) ** 0.5
                    distance = Config.FOCAL_LENGTH * Config.REAL_HEIGHT / person_height

                    frame_data.human_found = True
                    frame_data.distance = distance
                    frame_data.human_radar = radar * 2 - 1
                else:
                    frame_data.human_found = False
                    frame_data.human_radar = 0
                    frame_data.distance = 0

                if results.right_hand_landmarks is not None:
                    landmark_list = calc_landmark_list(
                        frame_data.frame, results.right_hand_landmarks
                    )

                    # Normalizing coordiantes
                    pre_processed_landmark_list = pre_process_landmark(landmark_list)

                    hand_gesture_id, confidence = self.gesture_classifier(
                        pre_processed_landmark_list
                    )
                    if confidence >= 0.8:
                        label = self.gesture_classifier_labels[hand_gesture_id]
                        frame_data.gesture = label
                        if self.stop_mode == False:
                            if label == "Open hand":
                                self.gesture_counter += 1
                            else:
                                self.gesture_counter = 0
                        else:
                            if label == "Thumb Up":
                                self.gesture_counter += 1
                            else:
                                self.gesture_counter = 0

                        if self.gesture_counter >= 10:
                            self.stop_mode = not self.stop_mode

                self.result_queue.put(frame_data)

                # Opcjonalna wizualizacja
                if self.enable_preview:
                    self.draw_visualization(frame_data)

    def data_analyzer(self):
        com = NucleoCom()
        v = 50
        w = 50
        mapping = {"OK": 2, "Peace": 3, "Pasta": 4}

        while not self.stop_event.is_set():
            try:
                result = self.result_queue.get(timeout=1)
                data = {
                    "frame_index": result.index,
                    "human_found": result.human_found,
                    "distance": round(result.distance, 2) if result.human_found else 0,
                    "human_radar": round(result.human_radar, 2)
                    if result.human_found
                    else 0,
                    "timestamp": time.time(),
                }

                print(data)
                l_speed = 0
                r_speed = 0
                gesture = mapping.get(result.gesture, 0)
                if self.stop_mode is True:
                    com.send_cmd()
                    continue
                if result.human_found is False:
                    com.send_cmd(matrix_num=1)
                    continue
                if abs(result.human_radar) > 0.2:
                    l_speed -= w * result.human_radar
                    r_speed += w * result.human_radar
                if result.distance > 100:
                    l_speed += v
                    r_speed += v
                print(l_speed, r_speed)
                com.send_cmd(matrix_num=gesture, rmot=r_speed, lmot=l_speed)

            except queue.Empty:
                continue

    def run(self):
        print(
            f"Program uruchomiony z {self.num_threads} wątkami obliczeniowymi. Naciśnij Ctrl+C aby zakończyć."
        )

        # Uruchomienie wątków
        generator = threading.Thread(target=self.frame_generator)
        analyzer = threading.Thread(target=self.data_analyzer)

        # Uruchomienie wątków przetwarzających
        for i in range(self.num_threads):
            process_thread = threading.Thread(target=self.process_frame, args=(i,))
            self.processing_threads.append(process_thread)
            process_thread.start()

        # Opcjonalne uruchomienie podglądu
        if self.enable_preview:
            preview_thread = threading.Thread(target=self.preview_thread)
            preview_thread.start()

        generator.start()
        analyzer.start()

        # Czekanie na sygnał zakończenia
        self.stop_event.wait()

        # Czekanie na zakończenie wątków
        print("\nKończenie pracy wątków...")
        generator.join()
        for thread in self.processing_threads:
            thread.join()
        analyzer.join()
        if self.enable_preview:
            preview_thread.join()
        print("Program zakończony.")


if __name__ == "__main__":
    vision_system = VisionSystem(
        num_threads=6, enable_preview=False
    )  # Możemy włączyć/wyłączyć podgląd
    vision_system.run()
