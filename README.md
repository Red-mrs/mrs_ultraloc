# mrs_ultraloc

Fuses a UVDAR bearing with a UWB range and publishes the resulting 3D position of each other vehicle.

The [UVDAR](https://github.com/ctu-mrs/uvdar_core) bearing endpoint already resolves a tracked blinker into a unit
bearing in a robot-fixed frame, and the [UWB driver](https://github.com/Red-mrs/uwb_driver) reports the distance to a
peer module. Neither alone localises the peer: the bearing gives direction without scale, the range gives scale
without direction. Combining them is one multiplication — but only once the two streams are attributed to the same
vehicle, which is what most of this package is about.

```
uvdar_core bearing_node ──BearingObservationArrayStamped──┐
                                                          ├─► uwb_uvdar_fusion_node ──FusionTargetArrayStamped──►
uwb_driver              ──UwbRangeStamped─────────────────┘
```

## Requirements

- ROS 2, developed on Jazzy.
- `uvdar_core` and `uwb_driver` built in the same workspace, or otherwise visible to `find_package()`. Both of their
  message types are consumed from the installed headers rather than copied here, so a build without them fails at the
  `find_package()` call with a named missing package.

Build the two producers before this package:

```bash
cd ~/ros2_ws
git clone https://github.com/ctu-mrs/uvdar_core           src/uvdar_core
git clone <uwb_driver>                                    src/uwb_driver
git clone <this repo>                                     src/mrs_ultraloc
colcon build --symlink-install --packages-up-to mrs_ultraloc
source install/setup.bash
```

The overlay has to be sourced in every shell that touches the topics, including the one inspecting them — the message
types here are named after this package, so `ros2 topic echo` in a plain shell reports them invalid.

## What the node does

Subscribes to both inputs, keeps the newest bearing per UVDAR signal id and the newest range per UWB address, and on a
timer combines the two for every configured pair:

```
position = bearing * range
```

The bearing is used as it arrives. `BearingObservationArrayStamped.header.frame_id` names the frame it is expressed in
— the camera's own frame for the current UVDAR config — and the fused position is published in that same frame. No
camera calibration is involved, because back-projection already happened upstream in `bearing_node`.

Publishing that frame unrotated is right for a horizontally mounted camera and wrong for a tilted one. Composing
several cameras, each with its own mounting transform, is the next step and is deliberately not done here; see
[Output frame](#output-frame).

Attribution is the substance of the node, so the details worth knowing:

- **Which peer a range belongs to.** A range report carries three addresses — initiator, responder, and the reporting
  module's own. The peer is whichever endpoint is not this vehicle. A report whose `own_address` is neither endpoint
  came from somebody else's module and is dropped, since it says nothing about our distance to anyone.
- **Which blinker belongs to that peer.** The `uwb_uvdar_id_pairs` parameter maps UWB address to UVDAR signal id, so
  the addressing scheme lives in config rather than in the code. Both sides of the list are checked for duplicates at
  startup: a repeated address or id would fuse two vehicles into one target, which is invisible downstream.
- **How fresh each side has to be.** Ranges arrive in bursts of a few hertz while bearings come continuously, so each
  sensor has its own timeout. A target is published only while both sides are inside theirs, which means a vehicle out
  of UWB range disappears from the output rather than being published at a stale distance.
- **Ranges out of the plausible window** are dropped before they are stored, so one multipath outlier cannot
  overwrite a good distance and sit there until the next report. The modules report invalid fixes as a short distance
  rather than suppressing them, so the lower bound is not only a sanity check.
- **Two observations of one id in one batch** are averaged as unit vectors rather than as components, which keeps the
  result a direction. This should not happen — the tracker resolves one track per decoded signal — but it costs nothing
  to tolerate.

### Output

[mrs_ultraloc/msg/FusionTargetArrayStamped.msg](msg/FusionTargetArrayStamped.msg) holds one
[FusionTarget](msg/FusionTarget.msg) per fused vehicle: the position, the range it came from, the bearing it came from,
per-side timestamps, and a 3×3 position covariance. Empty arrays are not published, so silence means nothing was fused
rather than that the node is not running.

The covariance is a first-order propagation: angular uncertainty scaled by the range squared, plus a radial term from
the range noise. Where the bearing message carries a usable covariance it is used; where it does not, `bearing_sigma_rad`
is spread over the tangent plane instead. The result is rank deficient by construction — the bearing constrains two
axes, the range the third.

Run `ros2 interface show mrs_ultraloc/msg/FusionTarget` for field-level documentation.

## Configuration

Everything is in [config/uwb_uvdar_fusion.yaml](config/uwb_uvdar_fusion.yaml), which documents each parameter.

That file groups its parameters under `/**` rather than under the node name. A parameter file is matched against a
node's fully qualified name, so with the node launched in a namespace — which the launch file does — a bare
`uwb_uvdar_fusion:` key matches nothing, and ROS reports nothing wrong with that: the node starts and runs on its code
defaults. Keep `/**` in a file of your own, or name the node in full (`/uav/uwb_uvdar_fusion:`).

The startup line is the quickest confirmation that a parameter file took effect; it names both inputs, the output and
the publish rate:

```
Fusing 4 UWB:UVDAR pair(s), '/uav/uvdar/bearing/camera_0/observations' + '/uav/uwb/distance' -> '/uav/uwb_uvdar_fusion/targets' at 20.0 Hz
```

The two input topics default to absolute names, because the producers namespace themselves independently of this node:
the bearing endpoint publishes under the UAV namespace from the UVDAR config file, and the UWB driver publishes
relative to the container it runs in. A relative default would resolve against whichever namespace this node lands in
and quietly subscribe to nothing.

```bash
ros2 launch mrs_ultraloc uwb_uvdar_fusion.launch.py
ros2 launch mrs_ultraloc uwb_uvdar_fusion.launch.py config:=/path/to/my_params.yaml
ros2 topic echo /uav/uwb_uvdar_fusion/targets
```

Against a bag, set `use_sim_time` so the freshness checks follow the recorded clock:

```bash
ros2 launch mrs_ultraloc uwb_uvdar_fusion.launch.py use_sim_time:=true
```

### Finding the id pairing

`uwb_uvdar_id_pairs` defaults to the four pairs the ROS 1 fusion used — `0xAA:28`, `0xBB:29`, `0xCC:30`, `0xDD:31`.
To check what a setup actually reports, watch both inputs side by side:

```bash
ros2 topic echo /uav/uwb/distance          # range.initiator/responder/own_address
ros2 topic echo /uav/uvdar/bearing/camera_0/observations   # observations[].id
```

If the pairing is wrong the node says which side it is missing rather than publishing nothing quietly:

```
Never received a UVDAR bearing for id 28 on '/uav/uvdar/bearing/camera_0/observations'; ...
No target fused: no bearing fresher than 1.0 s for id(s) 28,30, no range fresher than 2.0 s for address(es) 0xAA
```

A pair member that has never appeared at all is reported once, since that is a config error and repeating it adds
nothing. A member that reported and then went quiet is only reported while nothing at all is being fused, because a
teammate driving out of range is normal.

## Output frame

The position is published in the frame the bearing was stamped with, unrotated. For a camera looking horizontally
forward its optical frame differs from the body frame, so a tilted mount needs a transform that this node does not
apply.

That is the intended shape while there is one camera. With three cameras the fusion happens three times and the
mounting transforms belong to a later stage that combines them, at which point the frame to transform into becomes a
decision with a single right answer rather than a guess made here.
