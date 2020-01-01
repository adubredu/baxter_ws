import tkinter as tk, threading
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as Simage
from PIL import Image, ImageTk

video_name = "vid.mp4" #This is your video file path
video = cv2.VideoCapture(video_name)

class Vid:
    def __init__(self):
        self.current_image = None
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/rgb/image_raw', Simage, self.image_callback)
        self.root = tk.Tk()
        self.label = tk.Label(self.root)
        while not rospy.is_shutdown():
            self.stream(self.label)
            self.root.update()
        # self.root.mainloop()
        
        

    def image_callback(self, image):
            self.current_image = image

    def stream(self, label):
        # while not rospy.is_shutdown():
        if (self.current_image is not None):
            frame = self.bridge.imgmsg_to_cv2(self.current_image,'passthrough')      
            # cv2.imshow('test',frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            frame_image = ImageTk.PhotoImage(Image.fromarray(frame))
            self.label.config(image=frame_image)
            self.label.image = frame_image
            self.label.pack()
            print('yes')

        else:
            print('none')

if __name__ == "__main__":
    rospy.init_node('gui')
    vid = Vid()
    # root = tk.Tk()
    # my_label = tk.Label(root)
    # my_label.pack()
    # thread = threading.Thread(target=vid.stream, args=(my_label,))
    # thread.daemon = 1
    # thread.start()
    # root.mainloop()