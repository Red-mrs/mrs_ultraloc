#!/usr/bin/env python3

"""The three-camera rig: ``camera_left``, ``camera_right`` and ``camera_back``.

    ros2 launch mrs_ultraloc three_cams.launch.py dry_run:=true
    ros2 launch mrs_ultraloc three_cams.launch.py

Needs ``UWB_SERIAL``, ``CAMERA_LEFT``, ``CAMERA_RIGHT``, ``CAMERA_BACK`` and their
``EXPOSE_US_*`` in the shell, and a distinct serial per slot - two slots naming one
device is refused at startup rather than left to the driver.
``config/cameras.yaml`` holds the ``.bashrc`` block and the mounts.

Three cameras, still one bearing endpoint
-----------------------------------------

The count of processes does not go with the count of cameras, which surprises people and
is the reason this package has one bringup rather than three: ``uvdar_core``'s detector,
tracker and bearing each iterate over an ``inputs`` list, so three cameras are three
entries in one config and three publishers in one process. Only the camera drivers, the
mounts and the fusion nodes are started three times. ``launch/camera_rig.launch.py`` has
the table and the reasoning.

Three target streams, deliberately unmerged
-------------------------------------------

    /<uav>/camera_left/uwb_uvdar_fusion/targets
    /<uav>/camera_right/uwb_uvdar_fusion/targets
    /<uav>/camera_back/uwb_uvdar_fusion/targets

The same target seen by two cameras appears twice, in two frames, from two nodes. A
single stream for it would need the two bearings to be associated and then composed
through their mounts, and this package does neither: a UWB range says nothing about which
camera should be seeing the target, so which stream a range belongs to is a decision a
later stage has to make out loud.

Watching two at once is easiest through the visualiser, which takes one topic per
instance:

    ros2 launch mrs_ultraloc single_camera_marker.launch.py \\
        input_topic:=/uav/camera_left/uwb_uvdar_fusion/targets

The rest - the slot list, the arguments, why a wrapper at all - is one_cam.launch.py's
docstring, and the behaviour is launch/camera_rig.launch.py's.
"""

from ultraloc_tools.camera_rig import RIG_THREE
from ultraloc_tools.camera_rig_launch import rig_launch_description


def generate_launch_description():
    return rig_launch_description(RIG_THREE,
                                  'port side, starboard and aft')
