#!/usr/bin/env python3
import numpy as np
import math
import rospy
import sys
import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from ctl_msgs.msg import Aurora
from roscpp.srv import SetLoggerLevel
from sksurgerynditracker.nditracker import NDITracker


class Aurora_Sensor:
    def __init__(self) -> None:
        self._tag = self.__class__.__name__
        rospy.init_node('aurora_sensor_node')

        self._DEBUG = bool(rospy.get_param(f"{rospy.get_name()}/DEBUG_OUTPUT"))
        if self._DEBUG:
            rospy.wait_for_service(f"{rospy.get_name()}/set_logger_level")
            s = rospy.ServiceProxy(f"{rospy.get_name()}/set_logger_level", SetLoggerLevel)
            s.call('rosout', 'DEBUG')

        self._base_link_name = rospy.get_param(f"{rospy.get_name()}/base_link_name")
        markers = rospy.get_param(f"{rospy.get_name()}/ndi_sensor/marker")
        self._markers = {}
        for marker in markers:
            self._markers[marker['id']] = marker['name']

        self._br = tf2_ros.TransformBroadcaster()
        self._pub = rospy.Publisher(f"{rospy.get_name()}/sensor_data", Aurora, queue_size=10)
        self._aurora_sensor_msg = Aurora()

        rospy.loginfo(f"{self._tag} - __init__()")

        rospy.on_shutdown(self._shutdown)

    def _shutdown(self):
        rospy.loginfo(f"{self._tag} - _shutdown()")
        self.TRACKER.stop_tracking()
        self.TRACKER.close()

    def _receive_sensor_data(self):
        port_handles, timestamps, framenumbers, tracking, quality = self.TRACKER.get_frame()
        tracking = np.array(tracking)
        tracking[:, :3, 3:] = tracking[:, :3, 3:] / 1000
        trans = []

        for idx in range(len(port_handles)):
            if math.isnan(tracking[idx, 0, 0]) or timestamps[idx] == 0.0:
                continue
            self._aurora_sensor_msg.port_handles[idx] = port_handles[idx]
            if port_handles[idx] in self._markers:
                self._aurora_sensor_msg.marker_names[idx] = self._markers[port_handles[idx]]
            else:
                self._aurora_sensor_msg.marker_names[idx] = f"marker_{idx}"
            self._aurora_sensor_msg.timestamps[idx] = timestamps[idx]
            self._aurora_sensor_msg.framenumbers[idx] = framenumbers[idx]
            self._aurora_sensor_msg.quality[idx] = quality[idx]

            t = TransformStamped()
            t.header.stamp = rospy.Time.from_sec(timestamps[idx])
            t.header.frame_id = self._base_link_name
            t.child_frame_id = self._aurora_sensor_msg.marker_names[idx]
            t.transform.translation.x = tracking[idx, 0, 3]
            t.transform.translation.y = tracking[idx, 1, 3]
            t.transform.translation.z = tracking[idx, 2, 3]
            q = tf_conversions.transformations.quaternion_from_matrix(tracking[idx, :, :])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            trans.append(t)

            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation.w = t.transform.rotation.w
            p.orientation.x = t.transform.rotation.x
            p.orientation.y = t.transform.rotation.y
            p.orientation.z = t.transform.rotation.z
            self._aurora_sensor_msg.tracking[idx] = p

        return trans

    def execute(self):
        SETTINGS = {
            "tracker type": "aurora",
            "serial port": "/dev/ttyUSB0",
        }
        rospy.loginfo(f"{self._tag} - Create NDITracker object.")
        self.TRACKER = NDITracker(SETTINGS)
        rospy.loginfo(f"{self._tag} - Start Aurora Tracking.")
        self.TRACKER.start_tracking()
        rospy.loginfo(f"{self._tag} - Aurora Tracking is ready.")

        port_handles, timestamps, framenumbers, tracking, quality = self.TRACKER.get_frame()
        n_tools = len(port_handles)
        self._aurora_sensor_msg.port_handles = [0] * n_tools
        self._aurora_sensor_msg.marker_names = ['marker'] * n_tools
        self._aurora_sensor_msg.timestamps = [0.0] * n_tools
        self._aurora_sensor_msg.framenumbers = [0] * n_tools
        self._aurora_sensor_msg.tracking = [Pose()] * n_tools
        self._aurora_sensor_msg.quality = [0.0] * n_tools

        while not rospy.is_shutdown():
            try:
                self._br.sendTransform(self._receive_sensor_data())
                self._pub.publish(self._aurora_sensor_msg)
            except Exception as e:
                rospy.logerr(f"{self._tag} - Cannot read from the device: {e}")


if __name__ == '__main__':
    sensor = Aurora_Sensor()
    sensor.execute()