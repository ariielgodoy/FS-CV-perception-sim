from ultralytics import YOLO
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ConeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.detection_model = YOLO("/home/ariel/catkin_racecar_ws/src/fs_perception_pkg/models/best.pt")
        rospy.init_node("detection_node")
        self.detect = rospy.Subscriber("/camera/color/image_raw", Image, self.detect_cones, queue_size=1, buff_size=2**24)
        self.publish_detection = rospy.Publisher("/yolo/detection/image", Image)

    def detect_cones(self, image_to_predict):
        cv_image = self.bridge.imgmsg_to_cv2(image_to_predict, desired_encoding="bgr8")
        detection_result = self.detection_model(cv_image)
        detection_annotated = detection_result[0].plot(show=False) #Es una lista porque YOLO podria procesar mas de una imagen a la vez
        ros_msg = self.bridge.cv2_to_imgmsg(detection_annotated, encoding="bgr8")
        self.publish_detection.publish(ros_msg)

    def depth_operation(self, data):
        pass


def main():
    node = ConeDetector()
    rospy.spin()


if __name__ == '__main__':
    main()