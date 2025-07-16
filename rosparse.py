from mcap_ros2.reader import read_ros2_messages
import cv2
import numpy as np

camera_topic = "/sensor/camera_fr/compressed_image"

with open("/home/amrskk/.rviz2/intern_practice_ds.mcap", "rb") as f: #use your path to the dataset,there is mine
    for msg in read_ros2_messages(f, topics=[camera_topic]):
        img_bytes = msg.ros_msg.data
        np_arr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("Front Left Camera", img)
        if cv2.waitKey(1) == 27:  # Press ESC to quit
            break
