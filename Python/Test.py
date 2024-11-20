import cv2
from ultralytics import YOLO
from djitellopy import Tello
import numpy as np
import math
from time import sleep
import threading



mav = Tello()
mav.connect()
battery = mav.get_battery()
print(f"Battery level: {battery}%")
mav.streamon()


stop_streaming = False
stop_flying = False
found_target = False


# mav.takeoff()
# sleep(1)
# mav.rotate_clockwise(30)
# sleep(1)
# mav.land()

# mav.end()


# Load model
model = YOLO('D:\\IPV\\3ANO\\1SEMESTRE\\SE\\Trabalho\\Python\\BoxsV2.pt', task="detect")
# Get the 'BackgroundFrameRead' object that holds the latest captured frame from the mav
frame_read = mav.get_frame_read(with_queue=False, max_queue_len=0)

ROWS, COLS = (3, 3)


def draw_grid(img, h, w, color=(0, 255, 0), thickness=1):
    dy, dx = h / ROWS, w / COLS
    global command

    center_screen = int(w / 2), int(h / 2) - 50
    center_box = None
    msg = "NO OBJECT DETECTED"

    # draw vertical lines
    for x in np.linspace(start=dx, stop=w-dx, num=COLS-1):
        x = int(round(x))
        cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)

    # draw horizontal lines
    for y in np.linspace(start=dy, stop=h-dy, num=ROWS-1):
        y = int(round(y))
        cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)
        
    #Draw a circle in the middle of the image
    cv2.circle(img, center_screen, 10, (0,0,0), -1)

# Gives command to the drone based on the center of the image and the center of the bounding box
def give_comand(img, results, color=(0, 255, 0), thickness=1):
    global command
    global found_target
    h, w, _ = img.shape
    center_screen = int(w / 2), int(h / 2) - 50
    center_box = None
    msg = "NO OBJECT DETECTED"
    closer_box = 0

    draw_grid(img, h, w)

    for r in results:
        boxes = r.boxes
        # Find the box with the largest area using max
        if boxes:
            largest_box = max(
                boxes, 
                key=lambda box: (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1])
            )

            # Extract details of the largest box
            if largest_box:
                x1, y1, x2, y2 = largest_box.xyxy[0]
                center_x = int(x1 + (x2 - x1) / 2)
                center_y = int(y1 + (y2 - y1) / 2)
                center_box = (center_x, center_y)

                # Draw line from center of bounding box to center of screen
                cv2.line(img, center_box, center_screen, (255, 255, 255), 2)

    #cv2.putText(img, msg, (int(w / 2 - 64), 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    return img

def display():
    global stop_streaming
    global stop_flying
    while not stop_streaming:
        frame = frame_read.frame
        if frame is not None:
            # Color correct the image and run the model
            colored_frame =  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model(colored_frame, conf=0.6, verbose = False)
            
            annotated_frame = results[0].plot()
            final_frame = give_comand(annotated_frame, results)
            cv2.imshow("MAV STREAM", final_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_streaming = True
            stop_flying = True
            mav.streamoff()
            break


while not stop_flying:
    display()


# Land the drone safely
mav.end()
cv2.destroyAllWindows()