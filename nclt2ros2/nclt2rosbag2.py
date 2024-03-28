#!/usr/bin/env python

import rclpy

from nclt2ros2.converter.to_rosbag2 import ToRosbag2


def main():
    print("here")
    rclpy.init()

    # date = rclpy.get_param("~date", "2013-01-10")
    # bag_name = rclpy.get_param("~bag_name", "nclt")
    # cam_folder = rclpy.get_param("~cam_folder", 5)
    # lb3 = rclpy.get_param("~lb3", False)
    # sen = rclpy.get_param("~sen", False)
    # hokuyo = rclpy.get_param("~hokuyo", False)
    # vel = rclpy.get_param("~vel", False)
    # gt = rclpy.get_param("~gt", False)

    date = "2012-04-29"
    bag_name = "nclt"
    cam_folder = 5
    lb3 = False
    sen = False
    hokuyo = False
    vel = False
    gt = False

    converter = ToRosbag2(
        date=date,
        bag_name=bag_name,
        cam_folder=cam_folder,
        gt=gt,
        sen=sen,
        hokuyo=hokuyo,
        vel=vel,
        lb3=lb3,
    )
    converter.process()


if __name__ == "__main__":
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
