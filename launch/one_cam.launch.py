#!/usr/bin/env python3

"""The one-camera rig: ``camera``.

    ros2 launch mrs_ultraloc one_cam.launch.py dry_run:=true   # see what it decides
    ros2 launch mrs_ultraloc one_cam.launch.py                 # start it

Needs ``UWB_SERIAL``, ``CAMERA_MAIN`` and ``EXPOSE_US_MAIN`` in the shell;
``config/cameras.yaml`` holds the block to paste into ``~/.bashrc`` and says what happens
when one of them is missing.

Everything after the name of the camera - which serial, which calibration and mask, where
it is mounted, what the topics and TF frames are called, and how many processes come up -
comes from ``config/cameras.yaml`` through ``launch/camera_rig.launch.py``, which is where
the behaviour and its reasoning live.

Why this is a file and not an argument
--------------------------------------

``ros2 launch mrs_ultraloc camera_rig.launch.py cameras:=camera`` does the same thing. The
wrapper exists so that the rig a vehicle flies is a name rather than a command line: a
launch file called from a companion computer, a service unit or a teammate's shell history
has to be typed correctly every time, and one/two/three cameras is the thing that actually
differs between vehicles.

The slot list is ``RIG_ONE``, imported so this cannot disagree with the bringup it
includes, and ``cameras:=`` still overrides it per run - which is how a camera gets left
out for a bench test without touching the rig file.
"""

from ultraloc_tools.camera_rig import RIG_ONE
from ultraloc_tools.camera_rig_launch import rig_launch_description


def generate_launch_description():
    return rig_launch_description(RIG_ONE, 'the single camera, straight ahead')
