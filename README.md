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

One node per vehicle, two input topics, whatever the size of the camera rig. See
[One fusion node, one to three cameras](#one-fusion-node-one-to-three-cameras) for why several cameras are still one
input.

## Requirements

- ROS 2, developed on Jazzy.
- `uvdar_core` and `uwb_driver` built in the same workspace, or otherwise visible to `find_package()`. Both of their
  message types are consumed from the installed headers rather than copied here, so a build without them fails at the
  `find_package()` call with a named missing package.
- `uvdar_core` on branch **`ros2_bluefox_wip`**. This package's topic and frame names are that branch's — the bearing
  endpoint's `uvdar/bearing/observations` output and the `<uav>/bluefox_<slot>` camera frames. The `melodic-devel`
  naming (`uav13/rgb/bearing`, per-camera `camera_*` frames) does not produce a topic this node listens to, and a
  fusion node listening on a topic nobody publishes is not an error — see the startup line below.

Build the two producers before this package:

```bash
cd ~/ros2_ws
git clone -b ros2_bluefox_wip https://github.com/ctu-mrs/uvdar_core   src/uvdar_core
git clone <uwb_driver>                                               src/uwb_driver
git clone <this repo>                                                src/mrs_ultraloc
colcon build --symlink-install --packages-up-to mrs_ultraloc
source install/setup.bash
```

The overlay has to be sourced in every shell that touches the topics, including the one inspecting them — the message
types here are named after this package, so `ros2 topic echo` in a plain shell reports them invalid.

## Deployment: three sessions, three commands

This package starts the fusion node and nothing else. The cameras, the bearing stages and the UWB module are started by
their own packages, so a deployment is three shells — in tmux, in three splits, in three terminals:

```bash
# session 1 - the UWB module
ros2 launch uwb_driver uwb_driver.launch.py

# session 2 - three Bluefox cameras + detector + tracker + bearing endpoint
ros2 launch uvdar_core three_bluefox.launch.py

# session 3 - the fusion
ros2 launch mrs_ultraloc fusion.launch.py
```

[launch/fusion.launch.py](launch/fusion.launch.py) starts no camera, no detector, no tracker, no bearing stage and no
UWB driver. That is the whole of its relationship to the rest of the system, which is why it can be restarted after
changing the pairing without touching a camera that is already open.

All three read the same `$UAV_NAME` from `~/.bashrc`, and that single variable is the vehicle's namespace everywhere:

```bash
export UAV_NAME=uav13

# The UWB module's serial, as `ls /dev/serial/by-id` prints it. Read by
# uwb_driver.launch.py, which leaves its own config file in charge when unset.
export UWB_SERIAL=00AA11BB22CC

# One serial per camera, as `ros2 run bluefox2 bluefox2_list_cameras` prints it, and one
# exposure in microseconds per camera. Read by uvdar_core's three_bluefox.launch.py - see
# that file for the variables a smaller rig needs.
export BLUEFOX_LEFT_ID=25001879     EXPOSE_US_LEFT=2000
export BLUEFOX_RIGHT_ID=25001954    EXPOSE_US_RIGHT=2000
export BLUEFOX_BACK_ID=25002107     EXPOSE_US_BACK=3000
```

Avoid a UWB serial of the shape digits-E-digits (`206133814E31`). PyYAML correctly leaves such a token unquoted as a
string — YAML 1.1 needs a signed exponent to call it a number — but rclcpp's yaml-cpp reads that same token as a float,
and setting a float on a string parameter is fatal:

```
what():  parameter 'usb_serial' has invalid type: ... parameter {usb_serial} is of type {string},
         setting it to {double} is not allowed.
```

A purely numeric serial is quoted on the way out and is fine, as is anything whose letters come before the digits.

`uwb_driver` is deliberately not in the fusion launch file. It is one module per vehicle with a USB device of its own,
and it needs restarting far more often than a camera does — running it in its own session means you can pull it and
bring it back without a camera reopening.

For each session, what runs and what it publishes (with `UAV_NAME=uav13`):

```
camera    /uav13/left/bluefox/image_raw          node /uav13/left/bluefox
detector  /uav13/uvdar/left/detector/points_seen node /uav13/detector   (1 process, 3 inputs)
tracker   /uav13/uvdar/left/tracker/blinkers     node /uav13/tracker    (1 process, 3 inputs)
bearing   /uav13/uvdar/bearing/observations      node /uav13/bearing    (1 process, 1 publisher)
range     /uav13/uwb/distance                    uwb_driver
targets   /uav13/uwb_uvdar_fusion/targets        node /uav13/uwb_uvdar_fusion   <- in <uav>/fcu
```

The detector, tracker and bearing stages are **not** replicated per camera: `uvdar_core`'s stages each iterate over an
`inputs` list, so one process is how several cameras are meant to share a stage. Starting three detectors would open
one camera's image topic three times and put three publishers on one output topic.

## One fusion node, one to three cameras

There is one fusion node per vehicle and **one bearing topic per vehicle**, for a rig of one camera or three. That is
not a convention this package imposes — it is what `uvdar_core`'s bearing node does. It has a single publisher, on its
config's `bearing.output_topic`, and it looks up and applies the `camera_frame → output_frame` transform for every
input before it publishes, stamping the message with `output_frame`
([bearing_node.cpp](https://github.com/ctu-mrs/uvdar_core/blob/ros2_bluefox_wip/src/app/bearing_node.cpp) builds one
publisher from the section-level `output_topic` and never reads the per-input one). So a bigger rig **adds cameras to
that one topic** rather than adding topics. What the cameras' messages then look like to a consumer is the next section.

Which cameras exist is therefore a question about which `uvdar_core` config session 2 was started with — never about
anything in this package:

| uvdar_core launch | bearing config | cameras |
|---|---|---|
| `three_bluefox.launch.py` | `config/three_bluefox_bearing.yaml` | `left`, `right`, `back` |
| `two_bluefox.launch.py` + `bearing.launch.py` | `config/default_bearing.yaml` | `left`, `right` |
| `bearing.launch.py config_file:=<mine>.yaml` | whatever you pass | whatever it lists |

The fusion node takes one `bearing_topic`, which defaults to `<uav_name>/uvdar/bearing/observations`. That is the
`output_topic` of `three_bluefox_bearing.yaml`; note that `default_bearing.yaml` pins its output to the absolute
`/uav4/uvdar/bearing/observations`, so the two-camera config only matches this default on `uav4`. Pass
`bearing_topic:=` for any other vehicle.

A node whose bearing topic nothing publishes on fuses nothing and reports that once per paired id rather than refusing
to start, so the check after a deployment is the topic, not the log:

```bash
ros2 topic info /uav13/uvdar/bearing/observations    # 1 publisher, 1 subscriber
ros2 topic info /uav13/uwb/distance                  # 1 publisher, 1 subscriber
```

### Two cameras on one blinker: one topic, but not one merged reading

Worth reading before debugging a position that looks wrong, because "one topic for all cameras" is easy to over-read.

`onTrackerOutput` in `bearing_node.cpp` builds and publishes **one message per tracker callback**, and there is one
callback per camera input. So a blinker that two cameras can see arrives as **two separate messages**, one per camera,
whose relative order nothing fixes. It is not one batch containing two observations of one id.

What that means for the output:

- Within one message, observations of one id are averaged as unit vectors. That is the in-batch rule, and it is tested.
- **Across** messages there is no averaging: each message replaces the stored bearing for that id, so the newest camera
  wins outright.
- With `three_bluefox_bearing.yaml`'s `left` and `right` cameras — 140 mm apart, ±0.349 rad of splay — a vehicle close
  enough to be in both fields of view produces two bearings that disagree by degrees. The fused direction therefore
  **alternates between the two cameras at the tracker rate**, at the full amplitude of their disagreement, rather than
  settling between them.

`test_fusion.py::test_two_cameras_reporting_in_alternate_batches_overwrite` pins this behaviour down.

Why it is left alone here rather than smoothed: `BearingObservation` names no camera, so this node cannot tell which of
two sightings is the better one, and the endpoint does fill in each ray's `origin` — the camera's own position in the
output frame — but intersecting two offset rays to triangulate is a decision about the rig, not a transform, and it
belongs to whatever stage owns the camera geometry. A combination stage that wanted the intersection would need
per-camera attribution of some kind; note that the per-input `output_topic` entries in `default_bearing.yaml` look like
they provide it and do not, since that node never reads them, so those names are not topics that exist.

## What the node does

Subscribes to both inputs, keeps the newest bearing per UVDAR signal id and the newest range per UWB address, and on a
timer combines the two for every configured pair:

```
position = bearing * range
```

The bearing is used as it arrives. `BearingObservationArrayStamped.header.frame_id` names the frame it is expressed in
— `bearing.output_frame`, which the shipped configs set to `$UAV_NAME/fcu`, this vehicle's body frame — and the fused
position is published in that same frame. No camera calibration is involved, because back-projection already happened
upstream in `bearing_node`, and no TF is applied here, because the composing of the cameras into one frame already
happened there too, using the static mounts `three_bluefox.launch.py` publishes.

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
  result a direction. Note that this is a *within-batch* rule and that a multi-camera rig does not normally exercise it:
  each camera's ray arrives in its own batch, and the newest batch replaces the stored bearing. See
  [Two cameras on one blinker](#two-cameras-on-one-blinker-one-topic-but-not-one-merged-reading).

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

[launch/fusion.launch.py](launch/fusion.launch.py) is the only bringup this package has:

```bash
ros2 launch mrs_ultraloc fusion.launch.py
ros2 launch mrs_ultraloc fusion.launch.py uav_name:=uav13
ros2 launch mrs_ultraloc fusion.launch.py --show-args
ros2 topic echo /uav13/uwb_uvdar_fusion/targets
```

| argument | default | what it is |
|---|---|---|
| `uav_name` | `$UAV_NAME` | the vehicle namespace; prefixes both input topics as well as this node |
| `bearing_topic` | `uvdar/bearing/observations` | the bearing endpoint's one output; relative, so it resolves against `uav_name`, and an absolute name is passed through for a bag or a second endpoint |
| `uwb_topic` | `uwb/distance` | the UWB driver's range topic; one module per vehicle, so this never changes with the rig |
| `uwb_uvdar_id_pairs` | empty | comma-separated `0x13:3,0x15:0` override of the pairing in the file below |
| `fusion_config` | `config/uwb_uvdar_fusion.yaml` | a different parameter file |
| `use_sim_time` | `$USE_SIM_TIME` | follow `/clock`, for a bag |

`uwb_uvdar_id_pairs` is text rather than a list because a ROS 2 command line cannot express a list argument at all, and
an empty `[]` handed to rclcpp from a YAML file aborts the process — a sequence with no elements carries no element
type. The launch file splits it and hands the node a real list, and only when it is non-empty.

`uav_name` is also exported into the environment by the launch file, so a bearing or camera node started afterwards
from the same shell builds its frames under the namespace this file chose instead of a second `$UAV_NAME`.

Everything tunable is in [config/uwb_uvdar_fusion.yaml](config/uwb_uvdar_fusion.yaml), which documents each parameter.
The launch file overrides `bearing_topic`, `uwb_topic` and the pairing, because those depend on the arguments above and
on `$UAV_NAME`, which one file cannot state; every other value in it applies as written.

That file groups its parameters under `/**` rather than under the node name. A parameter file is matched against a
node's fully qualified name, so with the node launched in a namespace — which the launch file does — a bare
`uwb_uvdar_fusion:` key matches nothing, and ROS reports nothing wrong with that: the node starts and runs on its code
defaults. Keep `/**` in a file of your own, or name the node in full (`/uav/uwb_uvdar_fusion:`).

The startup line is the quickest confirmation that a parameter file took effect; it names both inputs, the output and
the publish rate:

```
Fusing 4 UWB:UVDAR pair(s), '/uav13/uvdar/bearing/observations' + '/uav13/uwb/distance' -> '/uav13/uwb_uvdar_fusion/targets' at 20.0 Hz
```

Read it, because a stale `bearing_topic` subscribes to nothing and the node's only further complaint is that no
bearing ever arrives.

### Finding the id pairing

`uwb_uvdar_id_pairs` defaults to the four pairs the ROS 1 fusion used — `0xAA:28`, `0xBB:29`, `0xCC:30`, `0xDD:31`.
To check what a setup actually reports, watch both inputs side by side:

```bash
ros2 topic echo /uav13/uwb/distance                       # range.initiator/responder/own_address
ros2 topic echo /uav13/uvdar/bearing/observations         # observations[].id
```

If the pairing is wrong the node says which side it is missing rather than publishing nothing quietly:

```
Never received a UVDAR bearing for id 28 on '/uav13/uvdar/bearing/observations'; ...
No target fused: no bearing fresher than 1.0 s for id(s) 28,30, no range fresher than 2.0 s for address(es) 0xAA
```

The topic named there is whatever `bearing_topic` was set to, so it is also the quickest way to see that a node is
listening where you think it is — and the way to tell "the pairing is wrong" from "the bearing endpoint is not
running", which otherwise look identical from the output.

A pair member that has never appeared at all is reported once, since that is a config error and repeating it adds
nothing. A member that reported and then went quiet is only reported while nothing at all is being fused, because a
teammate driving out of range is normal.

## Watching the output

[single_camera_marker.launch.py](launch/single_camera_marker.launch.py) draws fused targets as markers plus a trail:

```bash
ros2 launch mrs_ultraloc single_camera_marker.launch.py
```

Its rviz config pins `Fixed Frame` to `uav/fcu` — the frame the bearing endpoint publishes in. Set it to
`<uav_name>/fcu` for any other vehicle, or nothing resolves and RViz reports the frame as unknown while the markers are
being published normally.

## The cameras: what has to agree with what

This package owns none of this — it is `uvdar_core`'s side of the wiring — but a fused position that points the wrong
way is nearly always one of these, so it belongs in one place.

A camera is named in four places that cannot see each other, and a disagreement between them is not an error:

- The camera driver's `frame_id`, which `single_bluefox.launch.py` builds as `$UAV_NAME/bluefox_<slot>`.
- The mount that `three_bluefox.launch.py` publishes: `<uav>/fcu → <uav>/bluefox_<slot>`.
- That slot's `camera_frame` in the bearing config.
- Its `input_topic`, `<slot>/bluefox/image_raw` under the node namespace `<uav>/<slot>`.

The bearing node looks `camera_frame → output_frame` up in TF for every observation and **drops that camera's data**
with a throttled warning naming both frames if the lookup fails. Nothing fails at startup. So a mount that is missing,
or a frame spelled differently in two of those four places, is a camera that silently contributes nothing — while a
mount that is *present and wrong* is worse, because TF resolves, the targets move plausibly, and they are simply in the
wrong place. Every fused direction depends on it.

`ros2 run tf2_tools view_frames` after a start, and compare against a tape measure. The `back` mount in
`three_bluefox.launch.py` and the side-camera splay in `two_bluefox.launch.py` are the values to check first.

### Calibration files

The bearing config's per-input `calib_file` is what turns pixels into a unit bearing. `three_bluefox_bearing.yaml`
names `bf_left.yaml`, `bf_right.yaml` and `bf_back.yaml` under `camera/bluefox_ocam_calib/`; the first two ship with
`uvdar_core` and **`bf_back.yaml` does not exist yet**, so the third camera stops the bearing node at startup with a
message about the lens model. That is deliberate: pointing `back` at one of the two existing Bluefox calibrations would
produce bearings that are numerically plausible and wrong for this lens. Calibrate it (`calibrator.launch.py`) and put
the file at the path the config names.

A missing calibration file is the one case that fails loudly. A *wrong* one does not: `bearing_node` back-projects
through it into a unit bearing, so a wrong calibration yields bearings that are smooth, in range and consistently off.
Nor can [the simulator](#simulation) catch it — it publishes bearings directly and never opens a calibration file.

So the check needs the real camera and a known geometry: the static-hold test. Put a blinking target at a measured
position relative to the camera, hold it there, and compare the fused output against the measurement. The fused
`position` direction should match the direction to the target; its magnitude should match the UWB range, which it does
by construction, so the direction is the informative half. A target straight ahead at a known distance is the cheapest
first case, since a correct calibration must put it on the optical axis. What counts as close enough depends on the
mount tolerance, so it is worth doing once per camera and recording the residual rather than setting a threshold here
that this package cannot validate.

### No hardware attached

Which is the state most development happens in, and it looks like a broken bringup while being correct: the camera
processes fail to open their devices and publish nothing, so detector, tracker and bearing come up and stay silent, and
the fusion node logs `Never received a UVDAR bearing for id ...` at `diagnostics_rate_hz`. All of them staying up is
the pass condition. For a run that produces data, see [Simulation](#simulation).

## Simulation

[sim_fusion.launch.py](launch/sim_fusion.launch.py) stands two publishers in for the two sensors and starts a fusion
node beside them, so the whole path to a fused position can be exercised with nothing attached:

```bash
ros2 launch mrs_ultraloc sim_fusion.launch.py
ros2 topic echo /uav/uwb_uvdar_fusion/targets      # default uav_name is uav, not $UAV_NAME
```

One target, flying a rounded square roughly 1 to 5 m ahead. The positions are synthetic but arithmetically consistent —
`x ≈ range × bearing_x` on the output is the check that the pairing and the frame survived the launch. `--show-args`
covers the addresses, the signal id, the trajectory geometry, the frame and the rates.

The simulator imitates what the endpoint actually does: **one** bearing topic, stamped with `output_frame`, which
defaults here to `<uav_name>/fcu` — spelled from the producer's config rather than invented, because a simulator whose
topic has drifted from the producer's tests a wiring that no longer exists. There is no per-camera argument, because
there is no per-camera stream to pick.

To stand in for a rig that is really running, stop that rig's bearing stage and start the simulator alone, giving it
the rig's namespace and frame:

```bash
ros2 run mrs_ultraloc sim_uvdar_target.py --ros-args \
    -p topic:=/uav13/uvdar/bearing/observations \
    -p camera_frame:=uav13/fcu -p signal_id:=0
```

which leaves the real cameras and the real fusion node in place and replaces only the bearing source. `camera_frame`
matters: the fusion publishes in whatever frame the bearings carry, so a frame the TF tree does not know is not an
error, it is a target RViz draws at the origin.

Two instances of the *launch file* on one machine collide on node names and double-publish one targets topic — the
`ros2 run` form above is the one to use for anything additional.

## Output frame

Positions are published in the frame the bearing stream carries, unrotated. For the shipped configs that is
`bearing.output_frame` = `$UAV_NAME/fcu`, the vehicle's body frame, which is also the frame the camera mounts are
published against — so a fused target resolves against the rig's own TF tree with no work here.

A fused position in the wrong frame means the bearing endpoint's `output_frame`, not anything in this package. Two
things to check, in order: that the bearing node was started with a config whose `output_frame` names *this* vehicle
(`default_bearing.yaml` pins `uav4/fcu`, so for any other vehicle every observation is dropped with a throttled
warning — `three_bluefox_bearing.yaml` uses `$UAV_NAME/fcu` and works for any vehicle), and that the TF static mounts
for each `camera_frame` are actually running, since the rotation into `output_frame` happens in the bearing node using
them.
