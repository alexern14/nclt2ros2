import os
import rclpy
from nclt2ros2.definitions import ROOT_DIR


class BaseConvert:
    """Base Class for converting

    USAGE:
            BaseConvert(date='2013-01-10')

    """

    def __init__(self, date):
        self.date = date

        # create rosbag directory
        ROSBAG_PATH_DFLT = ROOT_DIR + "/nclt2ros/rosbags/"
        # self.rosbag_path = rclpy.get_param("~rosbag_output_path", ROSBAG_PATH_DFLT)
        self.rosbag_path = ROSBAG_PATH_DFLT

        if self.rosbag_path.endswith("/"):
            self.rosbag_dir = self.rosbag_path + str(self.date)
        else:
            self.rosbag_dir = self.rosbag_path + "/" + str(self.date)

        if not os.path.exists(self.rosbag_dir):
            os.makedirs(self.rosbag_dir)

        # create camera folder settings
        self.num_cameras = 6

        # init topic names
        self.gps_fix_topic = "/navsat/fix"
        self.gps_track_topic = "/navsat/track"
        self.gps_speed_topic = "/navsat/speed"
        self.gps_rtk_fix_topic = "/navsat/rtk/fix"
        self.gps_rtk_track_topic = "/navsat/rtk/track"
        self.gps_rtk_speed_topic = "/navsat/rtk/speed"
        self.imu_data_topic = "/imu/data"
        self.imu_mag_topic = "/imu/mag"
        self.odom_topic = "/odom"
        self.hokuyo_utm_topic = "/hokuyo_30m"
        self.hokuyo_urg_topic = "/hokuyo_4m"
        self.velodyne_topic = "/velodyne_points"
        self.ladybug_topic = "/images/raw"
        self.ground_truth_topic = "/ground_truth"

        # init frame ids
        self.gps_frame = "gps_link"
        self.gps_rtk_frame = "gps_rtk_link"
        self.imu_frame = "imu_link"
        self.odom_frame = "odom_link"
        self.hok_utm_frame = "laser_utm"
        self.hok_urg_frame = "laser_urg"
        self.velodyne_frame = "velodyne"
        self.ladybug_frame = "camera"
        self.ground_truth_frame = "ground_truth_link"
        self.body_frame = "base_link"

        # # init topic names
        # self.gps_fix_topic = rclpy.get_param("~gps_fix", "/navsat/fix")
        # self.gps_track_topic = rclpy.get_param("~gps_track", "/navsat/track")
        # self.gps_speed_topic = rclpy.get_param("~gps_speed", "/navsat/speed")
        # self.gps_rtk_fix_topic = rclpy.get_param("~gps_rtk_fix", "/navsat/rtk/fix")
        # self.gps_rtk_track_topic = rclpy.get_param(
        #     "~gps_rtk_track", "/navsat/rtk/track"
        # )
        # self.gps_rtk_speed_topic = rclpy.get_param(
        #     "~gps_rtk_speed", "/navsat/rtk/speed"
        # )
        # self.imu_data_topic = rclpy.get_param("~ms25_imu_data", "/imu/data")
        # self.imu_mag_topic = rclpy.get_param("~ms25_imu_mag", "/imu/mag")
        # self.odom_topic = rclpy.get_param("~wheel_odometry_topic", "/odom")
        # self.hokuyo_utm_topic = rclpy.get_param("~hokuyo_utm_topic", "/hokuyo_30m")
        # self.hokuyo_urg_topic = rclpy.get_param("~hokuyo_urg_topic", "/hokuyo_4m")
        # self.velodyne_topic = rclpy.get_param("~velodyne_topic", "/velodyne_points")
        # self.ladybug_topic = rclpy.get_param("~ladybug_topic", "/images/raw")
        # self.ground_truth_topic = rclpy.get_param(
        #     "~ground_truth_topic", "/ground_truth"
        # )

        # # init frame ids
        # self.gps_frame = rclpy.get_param("~gps_sensor", "gps_link")
        # self.gps_rtk_frame = rclpy.get_param("~gps_rtk_sensor", "gps_rtk_link")
        # self.imu_frame = rclpy.get_param("~imu_sensor", "imu_link")
        # self.odom_frame = rclpy.get_param("~wheel_odometry", "odom_link")
        # self.hok_utm_frame = rclpy.get_param("~hokuyo_utm_lidar", "laser_utm")
        # self.hok_urg_frame = rclpy.get_param("~hokuyo_urg_lidar", "laser_urg")
        # self.velodyne_frame = rclpy.get_param("~velodyne_lidar", "velodyne")
        # self.ladybug_frame = rclpy.get_param("~ladybug_sensor", "camera")
        # self.ground_truth_frame = rclpy.get_param("~ground_truth", "ground_truth_link")
        # self.body_frame = rclpy.get_param("~body", "base_link")
