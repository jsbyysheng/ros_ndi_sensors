#!/usr/bin/env python3
import rospy
from pathlib import Path
from sksurgerynditracker.nditracker import NDITracker

rospy.init_node('test_subst_value', anonymous=True)

print(rospy.get_name())
print(rospy.get_param(f"{rospy.get_name()}/tracker_type"))
print(rospy.get_param(f"{rospy.get_name()}/port"))
print(rospy.get_param(f"{rospy.get_name()}/base_link_name"))
print(rospy.get_param(f"{rospy.get_name()}/polaris_sensor/marker"))

# while not rospy.is_shutdown():
#     rospy.sleep(1.0)

roms_base = Path('/home/avadakedavra/Workspace/catkin_ws/src/drivers/ros_ndi_sensors/ros_ndi_tracker/config/roms')
roms = [str(rom) for rom in roms_base.glob('*.rom')]

SETTINGS = {
    "tracker type": "polaris",
    "serial port": "/dev/ttyUSB2",
    "romfiles" : roms
}

TRACKER = NDITracker(SETTINGS)
for idx, file in zip(*TRACKER.get_tool_descriptions()):
    rospy.loginfo(f'{idx} - {file}')

TRACKER.start_tracking()

while not rospy.is_shutdown():
    port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()
    rospy.loginfo_throttle(1.0, f'{port_handles}, {timestamps}, {framenumbers}, {quality}')

TRACKER.stop_tracking()
TRACKER.close()

'''
polaris: port_handles - rom_files
                1     - /path/to/apple05.rom
                2     - /path/to/apple03.rom
                3     - /path/to/apple06.rom
                4     - /path/to/apple04.rom
                5     - /path/to/apple02.rom

aurora: port_handles - marker_names
                10    - 0
                11    - 1
                12    - 2
                13    - 3
'''