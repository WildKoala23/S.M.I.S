import cv2
from ultralytics import YOLO
from djitellopy import Tello
import numpy as np
import math
from time import sleep
import threading    

# Load model
model = YOLO('C:\\Users\\pedrinho\\Documents\\GitHub\\S.M.I.S\\Python\\BoxsV2.pt', task="detect")

# Constants
ROWS, COLS = (3, 3)

stop_streaming = False
stop_flying = False
found_target = False
sent_command = False

# Initialize mav (Tello drone)
mav = Tello()
mav.connect()
battery = mav.get_battery()
print(f"Battery level: {battery}%")
mav.streamon()

# Get the 'BackgroundFrameRead' object that holds the latest captured frame from the mav
frame_read = mav.get_frame_read(with_queue=False, max_queue_len=0)

# Tells the drone what commadn to execute
command = 0

# Draw 3x3 grid
def draw_grid(img, h, w, color=(0, 255, 0), thickness=1):
    dy, dx = h / ROWS, w / COLS
    global command

    center_screen = int(w / 2), int(h / 2) - 50

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
    global sent_command
    h, w, _ = img.shape
    center_screen = int(w / 2), int(h / 2) - 50
    center_box = None
    draw_grid(img, h, w)


    largest_box = 0  # To hold the box with the largest area
    box_height = 0

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

                box_height = y2 - y1
                box_width = x2 - x1
                #box_area = int(box_height * box_width)
                #print(f'Area: {box_area}')
                # Draw line from center of bounding box to center of screen
                cv2.line(img, center_box, center_screen, (255, 255, 255), 2)
                #Draw a circle in the middle of the bounding box
                cv2.circle(img, center_box, 10, (0,0,0), -1)
        else:
            largest_box = 0
            break


    if center_box is not None:
        found_target = True
        # #If box is big, target is too close
        if box_height < 100:
            # Move forward
            command = 5
        elif box_height >= 250:
            #Move backward
            command = 6
        elif center_box[0] > (center_screen[0] + (w / 2) / 3):
            #msg = "GO LEFT"
            command = 1
        elif center_box[0] < (center_screen[0] - (w / 2) / 3):
            #msg = "GO RIGHT"
            command = 2
        elif center_box[1] + 20 < (center_screen[1] - (w / 2) / 3) :
            #msg = "GO UP"
            command = 3
        elif center_box[1] > (center_screen[1] + (w / 2) / 3):
            #msg = "GO DOWN"
            command = 4
    else:
        command = 0

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


# Function to make Drone spin to look for objects
def scann_room():
    while found_target is False:
        if battery <= 15:
            break
        mav.rotate_clockwise(25)


sleep(1)
# Start display thread
stream_thread = threading.Thread(target=display, daemon=True)
stream_thread.start()

sleep(5)
print("Taking off")
mav.takeoff()
sleep(1)

while not stop_flying:
    if command == 0:
        #scann_room()
        # found_target = False
        pass
    elif command == 1:
        mav.rotate_clockwise(20)
    elif command == 2:
        mav.rotate_counter_clockwise(20)
    elif command == 3:
        mav.move_up(20)
    elif command == 4:
        mav.move_down(20)
    elif command == 5:
        mav.move_forward(30)
    elif command == 6:
        mav.move_back(30)
    command = 0 
    sleep(1)
    
    

print(f"Battery level: {battery}%")
mav.land()
mav.end()
cv2.destroyAllWindows()