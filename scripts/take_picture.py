import rospy
import cv2
import cv_bridge
import numpy as np
import os
import time
from sensor_msgs.msg import Image

# -- constants --
MODE_BY_KEY = 0 # MODE 1
MODE_CONTINUOUS = 1 # MODE 2
MODE_CONTINUOUS_DURATION_SECOND = 1.25 # we define in MODE 2, we take a picture once per some seconds

# -- globals --
save_dir = ""
left_dir = ""
right_dir = ""
image_topic = ""
image = None

# -- callbacks --
def cb(msg):
    global image

    counter = msg.header.seq
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, "passthrough")
    
def main():
    global save_dir
    global left_dir
    global right_dir
    global image_topic
    global image
    
    rospy.init_node("take_picture_node")
    
    # -- retreival of parameters --
    image_topic = rospy.get_param("~image_topic")
    print("--> Subscribing to image topic {}".format(image_topic))
    
    save_dir = rospy.get_param("~save_dir")
    
    mode = rospy.get_param("~mode")
    print("--> Mode {}".format(mode))
    if mode != MODE_BY_KEY and mode != MODE_CONTINUOUS:
        print("--> Invalid mode")
        return
    
    # -- create necessary folders --
    # -- verify if the passed parent_folder argument is indeed a valid directory --
    left_dir = os.path.join(save_dir, "left")
    right_dir = os.path.join(save_dir, "right")
    print('--> Creating folders \n\t[%s] \n\tand \n\t[%s]' % (left_dir, right_dir))
    os.makedirs(right_dir, exist_ok=True)
    os.makedirs(left_dir, exist_ok=True)

    # -- we ask user to indicate the start timing of picture taking when in MODE 2 --
    if mode == MODE_CONTINUOUS:
        print("--> Please press any key to start taking pictures, the process will start 2 seconds after you press any key.")
        k = input()
        time.sleep(2)
    else:
        print("--> Press 'c' to take a picture.")
    print("--> Once the process started, press 'q' to stop the process.")
    
    sub = rospy.Subscriber(image_topic, Image, cb)

    left = np.zeros((480, 640))
    right = np.zeros((480, 640))
    last_taken = time.time()
    pic_cnt = -1
    while not rospy.is_shutdown():
        if image is None:
            cv2.imshow('left', left)
            cv2.imshow('right', right)
            k = cv2.waitKey(1)
            if k == ord('q'):
                break
            continue
        
        left = image[:, 0:640, :]
        right = image[:, 640:, :]
        
        cv2.imshow('left', left)
        cv2.imshow('right', right)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        
        duration = time.time() - last_taken
        
        # -- take picture --:
        #   * when in MODE 1, by pressing the 'c' key
        #   * when in MODE 2, once per a time specified by MODE_CONTINUOUS_DURATION_SECOND
        is_shooting = key == ord('c') if mode==MODE_BY_KEY else duration>=MODE_CONTINUOUS_DURATION_SECOND
        
        if is_shooting:
            last_taken = time.time()
            pic_cnt += 1

            print("--> Took a picture, current #pictures:", pic_cnt)

            cv2.imwrite(os.path.join(left_dir, str(pic_cnt) + '.png'), left)
            cv2.imwrite(os.path.join(right_dir, str(pic_cnt) + '.png'), right)
    
if __name__ == "__main__":
    main()