import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread

class VideoStreamThread(Thread):
    def __init__(self, topic_name):
        super(VideoStreamThread, self).__init__()
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('udp://@0.0.0.0:8554')
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=10)
        self.running = True

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                try:
                    ros_image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image_msg.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(0.5)
                    self.publisher.publish(ros_image_msg)

                    # cv2.imshow('Gopro Video Stream', frame)
                    # cv2.waitKey(1)
                except Exception as e:
                    rospy.logerr(e)

    def stop(self):
        self.running = False
        self.join()
        self.cap.release()

def main():
    rospy.init_node('udp_video_stream_publisher', anonymous=True)
    topic_name = 'left_camera'
    video_thread = VideoStreamThread(topic_name)
    video_thread.start()

    rospy.spin()

    video_thread.stop()

if __name__ == '__main__':
    main()
