#!/usr/bin/env python3

"""The two-camera rig: ``camera_front`` and ``camera_back``.

    ros2 launch mrs_ultraloc two_cams.launch.py dry_run:=true
    ros2 launch mrs_ultraloc two_cams.launch.py

Needs ``UWB_SERIAL``, ``CAMERA_FRONT``, ``CAMERA_BACK`` and their ``EXPOSE_US_*`` in the
shell; ``config/cameras.yaml`` holds the block to paste into ``~/.bashrc``.

Forward and aft, not left and right
-----------------------------------

A left/right pair on one vehicle buys coverage of the sides and costs a mount separation
that has to be measured to the centimetre before either bearing is worth fusing. Front and
aft buys the question a single forward camera cannot answer - is the target in front of me
or behind me - and the two fields of view do not have to overlap for that. This is also
what uvdar_core's own example mount turns out to be, whatever its ``left``/``right`` names
suggest: its second camera is yawed 160 degrees aft. See the comment on those mounts in
``config/cameras.yaml``.

Each camera gets its own fusion node, so this rig publishes two target streams:

    /<uav>/camera_front/uwb_uvdar_fusion/targets
    /<uav>/camera_back/uwb_uvdar_fusion/targets

Nothing merges them. The node fuses one bearing topic with one range topic
(``bearing_topic`` is a single string parameter), and its header records that composing
several cameras needs a mounting transform it deliberately does not apply
(include/mrs_ultraloc/uwb_uvdar_fusion_node.h:35-37). Beyond that: a UWB range says nothing
about which camera ought to be seeing the target, so deciding which stream a range belongs
to is a decision a later stage has to make explicitly, not something either node here can
infer.

The rest - the slot list, the arguments, why a wrapper at all - is one_cam.launch.py's
docstring, and the behaviour is launch/camera_rig.launch.py's.
"""

from ultraloc_tools.camera_rig import RIG_TWO
from ultraloc_tools.camera_rig_launch import rig_launch_description


def generate_launch_description():
    return rig_launch_description(RIG_TWO, 'the forward and the aft camera')
