import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


# import tf


class T265_Stereo:

    def __init__(self):

        self.fisheye_left = rospy.Subscriber("/camera/fisheye1/image_raw", Image,
                                             self.fisheye_left_callback, queue_size=10)
        self.fisheye_right = rospy.Subscriber("/camera/fisheye2/image_raw", Image,
                                              self.fisheye_right_callback, queue_size=10)

        self.cam_info_1 = rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo,
                                           self.fisheye_left_cam_info_callback)
        self.cam_info_2 = rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo,
                                           self.fisheye_right_cam_info_callback)

        self.pub_left_rect = rospy.Publisher("/t265_rectify/image_left_rect", Image, queue_size=1)
        self.pub_right_rect = rospy.Publisher("/t265_rectify/image_right_rect", Image, queue_size=1)

        self.bridge = CvBridge()

        self.initialized = False

        self.fisheye_left_img = None
        self.fisheye_right_img = None
        self.fisheye_left_cam_info = None
        self.fisheye_right_cam_info = None

        self.left_K = None
        self.right_K = None
        self.left_D = None
        self.right_D = None

        self.R = None
        self.T = None

        self.width = None
        self.height = None

        self.window_size = 5
        self.min_disp = 0
        # must be divisible by 16
        self.num_disp = 112 - self.min_disp
        self.max_disp = self.min_disp + self.num_disp

        self.got_fisheye_left_img = False
        self.got_fisheye_right_img = False
        self.got_fisheye_left_cam_info = False
        self.got_fisheye_right_cam_info = False

    def fisheye_left_callback(self, img_msg):
        print("left callback")

        self.fisheye_left_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        self.got_fisheye_left_img = True
        if self.initialized:

            undist_left = cv2.remap(src=self.fisheye_left_img,
                                    map1=self.undistort_rectify["left"][0],
                                    map2=self.undistort_rectify["left"][1],
                                    interpolation=cv2.INTER_LINEAR)
            color_image_left_msg = self.bridge.cv2_to_imgmsg(undist_left, encoding="mono8")
            self.pub_left_rect.publish(color_image_left_msg)

    def fisheye_right_callback(self, img_msg):
        print("right callback")
        self.fisheye_right_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        self.got_fisheye_right_img = True
        if self.initialized:
            undist_right = cv2.remap(src=self.fisheye_right_img,
                      map1=self.undistort_rectify["right"][0],
                      map2=self.undistort_rectify["right"][1],
                      interpolation=cv2.INTER_LINEAR)
            color_image_right_msg = self.bridge.cv2_to_imgmsg(undist_right, encoding="mono8")
            self.pub_right_rect.publish(color_image_right_msg)

    def fisheye_left_cam_info_callback(self, cam_info_msg):
        # print("left caminfo callback")
        self.fisheye_left_cam_info = cam_info_msg
        self.got_fisheye_left_cam_info = True
        # self.run()

    def fisheye_right_cam_info_callback(self, cam_info_msg):
        # print("right caminfo callback")
        self.fisheye_right_cam_info = cam_info_msg
        self.got_fisheye_right_cam_info = True

        if self.got_fisheye_left_img and self.got_fisheye_right_img and \
                self.got_fisheye_left_cam_info and self.got_fisheye_right_cam_info:
            if not self.initialized:
                self.initialize_matrices()
        # self.run()

    def rectify(self):
        center_undistorted = {"left": cv2.remap(src=self.fisheye_left_img,
                                                map1=self.undistort_rectify["left"][0],
                                                map2=self.undistort_rectify["left"][1],
                                                interpolation=cv2.INTER_LINEAR),
                              "right": cv2.remap(src=self.fisheye_right_img,
                                                 map1=self.undistort_rectify["right"][0],
                                                 map2=self.undistort_rectify["right"][1],
                                                 interpolation=cv2.INTER_LINEAR)}

        color_image_left = cv2.cvtColor(center_undistorted["left"][:, self.max_disp:],
                                        cv2.COLOR_GRAY2RGB)
        color_image_right = cv2.cvtColor(center_undistorted["right"][:, self.max_disp:],
                                         cv2.COLOR_GRAY2RGB)

        color_image_left_msg = self.bridge.cv2_to_imgmsg(color_image_left, encoding="rgb8")
        self.pub_left_rect.publish(color_image_left_msg)

        color_image_right_msg = self.bridge.cv2_to_imgmsg(color_image_right, encoding="rgb8")
        self.pub_right_rect.publish(color_image_right_msg)

    def initialize_matrices(self):
        self.left_K = np.array(self.fisheye_left_cam_info.K).reshape((3, 3))
        self.right_K = np.array(self.fisheye_right_cam_info.K).reshape((3, 3))

        self.left_D = np.array(self.fisheye_left_cam_info.D[:4])
        self.right_D = np.array(self.fisheye_right_cam_info.D[:4])

        (self.width, self.height) = (
        self.fisheye_left_cam_info.width, self.fisheye_right_cam_info.height)

        self.R = np.array([[0.99996531, 0.00580445, -0.00597129],
                           [-0.00579029, 0.99998039, 0.00238675],
                           [0.00598503, -0.00235209, 0.99997938]])

        self.T = np.array([-0.06423119, 0.00062814, 0.00013314])

        self.stereo_fov_rad = 90 * (np.pi / 180)  # 90 degree desired fov
        self.stereo_height_px = 300  # 300x300 pixel stereo output
        self.stereo_focal_px = self.stereo_height_px / 2 / np.tan(self.stereo_fov_rad / 2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras
        self.R_left = np.eye(3)
        self.R_right = self.R

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        self.stereo_width_px = self.stereo_height_px + self.max_disp
        self.stereo_size = (self.stereo_width_px, self.stereo_height_px)
        self.stereo_cx = (self.stereo_height_px - 1) / 2 + self.max_disp
        self.stereo_cy = (self.stereo_height_px - 1) / 2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
        self.P_left = np.array([[self.stereo_focal_px, 0, self.stereo_cx, 0],
                                [0, self.stereo_focal_px, self.stereo_cy, 0],
                                [0, 0, 1, 0]])
        self.P_right = self.P_left.copy()
        self.P_right[0][3] = self.T[0] * self.stereo_focal_px

        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        Q = np.array([[1, 0, 0, -(self.stereo_cx - self.max_disp)],
                      [0, 1, 0, -self.stereo_cy],
                      [0, 0, 0, self.stereo_focal_px],
                      [0, 0, -1 / self.T[0], 0]])

        m1type = cv2.CV_32FC1
        (self.lm1, self.lm2) = cv2.fisheye.initUndistortRectifyMap(self.left_K, self.left_D,
                                                                   self.R_left,
                                                                   self.P_left, self.stereo_size,
                                                                   m1type)

        (self.rm1, self.rm2) = cv2.fisheye.initUndistortRectifyMap(self.right_K, self.right_D,
                                                                   self.R_right,
                                                                   self.P_right, self.stereo_size,
                                                                   m1type)
        self.undistort_rectify = {"left": (self.lm1, self.lm2),
                                  "right": (self.rm1, self.rm2)}

        self.initialized = True

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.got_fisheye_left_img and self.got_fisheye_right_img and \
                    self.got_fisheye_left_cam_info and self.got_fisheye_right_cam_info:
                if not self.initialized:
                    self.initialize_matrices()
                self.rectify()
            # rate.sleep()


if __name__ == '__main__':
    rospy.init_node('T265_Rectify', anonymous=True)
    st = T265_Stereo()
    rospy.spin()
