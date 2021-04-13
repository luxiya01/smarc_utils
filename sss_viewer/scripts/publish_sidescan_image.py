#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from smarc_msgs.msg import Sidescan

class SidescanPublisher:
    """Subscribe to /vehicle/payload/sidescan message and publish the
    corresponding waterfall image to /vehicle/payload/sidescan/CompressedImage"""

    def __init__(self, width_in=1000, width_out=256, height=1000):
        self.bridge = CvBridge()
        self.width_in = width_in
        self.width_out = width_out
        self.height = height
        self.img = np.zeros((self.height, 2 * self.width_in), dtype=np.ubyte)
        self.sidescan_sub = rospy.Subscriber("/sam/payload/sidescan", Sidescan,
                self.callback)
        self.sidescan_image_pub = rospy.Publisher("/sam/payload/sidescan/CompressedImage", CompressedImage)

    def callback(self, msg):
        port = np.array(bytearray(msg.port_channel), dtype=np.ubyte)
        stbd = np.array(bytearray(msg.starboard_channel), dtype=np.ubyte)
        meas = np.concatenate([np.flip(port), stbd])
        self.img[1:, :] = self.img[:-1, :]
        self.img[0, :] = meas
        self.publish_image()

    def publish_image(self):
        resized_img = cv2.resize(self.img, (2*self.width_out, self.height),
                interpolation=cv2.INTER_AREA)
        try:
            imgmsg = self.bridge.cv2_to_compressed_imgmsg(resized_img,
                    "passthrough")
            self.sidescan_image_pub.publish(imgmsg)
        except CvBridgeError as e:
            print(f"Converting numpy image to imgmsg: {e}")


def main():
    node_name = "sidescan_image_publisher"
    rospy.init_node(node_name, anonymous=True)
    rospy.Rate(5)
    publisher = SidescanPublisher()

    print(f"Running {node_name}...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()
