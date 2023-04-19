from pathlib import Path
from sksurgerynditracker.nditracker import NDITracker

roms_base = Path('/home/avadakedavra/Workspace/catkin_ws/src/drivers/ros_ndi_sensors/ros_ndi_tracker/config/roms')
roms = [str(rom) for rom in roms_base.glob('*.rom')]

SETTINGS = {
    "tracker type": "polaris",
    "serial port": "/dev/ttyUSB2",
    "romfiles" : roms
}

TRACKER = NDITracker(SETTINGS)

TRACKER.start_tracking()
for i in range(100):
    port_handles, timestamps, framenumbers, tracking, quality = TRACKER.get_frame()
    for t in tracking:
        print(t)

TRACKER.stop_tracking()
TRACKER.close()