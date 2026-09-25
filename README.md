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

Publishing that frame unrotated is right for a horizontally mounted camera and wrong for a tilted one. The rig bringup
publishes each camera's mount as a TF static transform, so the transform is available to whoever needs it; applying it
is still not this node's job, and what that leaves open is in [Output frame](#output-frame).

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

`camera_0` in that line is the shipped default in [config/uwb_uvdar_fusion.yaml](config/uwb_uvdar_fusion.yaml), which
predates the rig slot names; a rig-started node reports the slot instead, e.g.
`'/uav/uvdar/bearing/camera_left/observations' ... -> '/uav/camera_left/uwb_uvdar_fusion/targets'`. If you launch this
node by hand against a running rig, set `bearing_topic` to the slot you want — a stale `camera_0` subscribes to nothing
and the node's only complaint is that no bearing ever arrives.

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

On a vehicle this node is normally started for you, once per camera, by [the rig bringup](#a-vehicle-one-or-several-cameras),
which passes each camera's own bearing topic. The launch file above is for running the node on its own against topics
that already exist.

### Finding the id pairing

`uwb_uvdar_id_pairs` defaults to the four pairs the ROS 1 fusion used — `0xAA:28`, `0xBB:29`, `0xCC:30`, `0xDD:31`.
To check what a setup actually reports, watch both inputs side by side, on the slot you are looking at:

```bash
ros2 topic echo /uav/uwb/distance                        # range.initiator/responder/own_address
ros2 topic echo /uav/uvdar/bearing/camera/observations   # observations[].id
```

If the pairing is wrong the node says which side it is missing rather than publishing nothing quietly:

```
Never received a UVDAR bearing for id 28 on '/uav/uvdar/bearing/camera_0/observations'; ...
No target fused: no bearing fresher than 1.0 s for id(s) 28,30, no range fresher than 2.0 s for address(es) 0xAA
```

The topic named there is whatever `bearing_topic` was set to, so it is also the quickest way to see that a node is
listening on the slot you think it is.

A pair member that has never appeared at all is reported once, since that is a config error and repeating it adds
nothing. A member that reported and then went quiet is only reported while nothing at all is being fused, because a
teammate driving out of range is normal.

## A vehicle: one or several cameras

Three launch files, one per vehicle:

```bash
ros2 launch mrs_ultraloc one_cam.launch.py        # camera
ros2 launch mrs_ultraloc two_cams.launch.py       # camera_front, camera_back
ros2 launch mrs_ultraloc three_cams.launch.py     # camera_left, camera_right, camera_back
```

They differ only in which slots they ask for; all three include
[launch/camera_rig.launch.py](launch/camera_rig.launch.py), which is what actually decides what runs. That file is also
the quickest way to look at a rig, and the form to use before the hardware is on the bench:

```bash
ros2 launch mrs_ultraloc camera_rig.launch.py cameras:=camera_left,camera_right dry_run:=true
```

`dry_run:=true` resolves everything, prints it, generates the UVDAR config, and starts nothing. Note that
`--print-description` is not a substitute: the whole bringup lives in one `OpaqueFunction` because the arguments are
cross-dependent, so `--print-description` prints an opaque placeholder and resolves none of it.

### What runs

For N cameras:

| stage | processes | namespace |
|---|---|---|
| bluefox2 camera | **N** | `<uav>/<slot>`, node `bluefox_<slot>` |
| static camera mount | **N** | `<uav>/fcu` → `<uav>/<slot>` |
| detector | 1 | `<uav>` — N inputs in one process |
| tracker | 1 | `<uav>` |
| bearing | 1 | `<uav>` — N publishers in one process |
| uwb_driver | **1** | `<uav>` — one module, however many cameras |
| fusion node | **N** | `<uav>/<slot>` |

The chain is not replicated per camera, and that is not a shortcut taken here: `uvdar_core`'s detector, tracker and
bearing each iterate over an `inputs` list, so one process is how several cameras are meant to share a stage. Starting
N detectors would open one camera's image topic N times and put N publishers on one output topic. One UWB module
serves every camera, so it is started once. Fusion *is* per-camera, because the node takes one `bearing_topic` and
publishes in the frame that bearing was stamped with — so each camera gets its own
`<uav>/<slot>/uwb_uvdar_fusion/targets` and nothing merges them. See [Output frame](#output-frame) for what that leaves
open.

### What you configure

Everything structural is in [config/cameras.yaml](config/cameras.yaml): which environment variable each slot reads, the
file-name patterns, and the mount. The per-vehicle facts go in `~/.bashrc`, because they belong to the airframe and
change when you swap a camera:

```bash
export UAV_NAME=uav13
export UWB_SERIAL=00AA11BB22CC

# one serial per camera, as `ros2 run bluefox2 bluefox2_list_cameras` prints it, and one
# exposure in microseconds per camera. Only the variables the launched rig reads are
# used, so setting all five on a one-camera vehicle is harmless - which is also why
# each one appears exactly once below, whatever rig you fly.
export CAMERA_MAIN=25001879     EXPOSE_US_MAIN=2000     # one_cam
export CAMERA_FRONT=25001879    EXPOSE_US_FRONT=2000    # two_cams
export CAMERA_BACK=25002107     EXPOSE_US_BACK=3000     # two_cams, three_cams
export CAMERA_LEFT=25001879     EXPOSE_US_LEFT=2000     # three_cams
export CAMERA_RIGHT=25001954    EXPOSE_US_RIGHT=2000    # three_cams

# optional; defaults to $UAV_NAME with the leading "uav" removed, so UAV_NAME=uav13
# already means MRS_ID=13, which the mask file names below use
# export MRS_ID=13
```

The serials are placeholders, but the three distinct ones are load-bearing in a way the numbers are not:

- **Each camera in a rig needs a different serial.** Two slots naming one serial is one device asked for by two nodes,
  and the launch refuses rather than letting the second camera fail to open it — so `camera_back` needs a serial of its
  own, and a calibration file for it, which is the other thing a camera costs.
- **For the UWB module, avoid a serial of the shape digits-E-digits.** Launch writes the serial through PyYAML, which
  correctly leaves `206133814E31` unquoted as a string — YAML 1.1 needs a signed exponent to call it a number — but
  rclcpp's yaml-cpp reads that same token as a float, and setting a float on a string parameter is fatal:

  ```
  what():  parameter 'usb_serial' has invalid type: ... parameter {usb_serial} is of type {string},
           setting it to {double} is not allowed.
  ```

  A purely numeric serial (`25001879`) is quoted on the way out and is fine, as is anything whose letters come before
  the digits (`00AA11BB22CC`). Nothing here checks or works around it yet — see the note in
  [config/cameras.yaml](config/cameras.yaml). The camera side takes the same token through the same path and has been
  seen to survive it, reporting an unknown serial as `Requested device with serial ... not found`, but no reason for
  the difference has been established, so treat that as an observation rather than a guarantee.

Slot names are the TF frame suffix and the topic component at once, so a rig is named in one place. Which slots each
launch file asks for:

| launch file | `cameras:=` | frames | fusion namespaces |
|---|---|---|---|
| `one_cam` | `camera` | `<uav>/camera` | `<uav>/camera` |
| `two_cams` | `camera_front,camera_back` | `<uav>/camera_front`, `<uav>/camera_back` | same, per slot |
| `three_cams` | `camera_left,camera_right,camera_back` | `<uav>/camera_left`, … | same, per slot |

For each slot `<slot>`, with the single-camera rig as the example — this is what `dry_run` prints, and what to subscribe
to:

```
image     /uav13/camera/bluefox_camera/image_raw
detector  /uav13/uvdar/detector/camera/points_seen
tracker   /uav13/uvdar/tracker/camera/blinkers
bearing   /uav13/uvdar/bearing/camera/observations
targets   /uav13/camera/uwb_uvdar_fusion/targets   <- bearing + /uav13/uwb/distance
```

Other arguments worth knowing, on `camera_rig.launch.py` — the three wrappers forward them but declare only the six
above, so `--show-args` on a wrapper lists those six and not these: `keep_uvdar_config:=/path/out.yaml` to keep the
generated UVDAR config instead of a temp file, `camera_stagger_sec` for the delay between camera processes,
`uwb_serial` to override `$UWB_SERIAL`, and `uvdar_base_config` for the uvdar_core file the generated one is seeded
from. `fusion_config:=/path/fusion.yaml` is the one that reaches tuning: every value in it applies to every camera,
except `bearing_topic` and `uwb_topic`, which are per-camera and so are set by the bringup regardless.
`standalone:=false` runs the UWB driver as a component; leave it at its default unless you know why — see the note on
it in that file's `--show-args`.

### Before the first real start

Two files are looked up by name derived from the serial — patterns under `paths` in `config/cameras.yaml`, where
`{uvdar_config}` is `uvdar_core`'s installed `config` directory and a relative path resolves against the rig file's own
directory. `dry_run` prints both fully resolved:

```
calib  <uvdar_core install>/config/camera/bluefox_ocam_calib/bf_uv_<serial>.yaml
mask   <this package>/share/mrs_ultraloc/config/mask/<MRS_ID>_<serial>.png
```

A missing calibration stops a real start and names the path it wanted — deliberately, because a wrong calibration
produces numerically plausible bearings, so "it runs" is not evidence the file was right. A missing mask downgrades to
one warning per camera and turns masking off. So:

- Copy each camera's calibration to `bf_uv_<serial>.yaml`, or repoint `paths.calib_file`. Only `bf_uv_25001879.yaml`
  and `bf_uv_25001954.yaml` ship with `uvdar_core`, and they are not close: their principal points are 53 and 42 pixels
  apart on a 752×480 image, and every polynomial differs. Swapping one for the other is not a small error, and nothing
  detects it — see the check below.
- `config/mask/` does not exist in the repo yet; create it when you have the masks.

#### Checking that a calibration file is the right one

The launch can only check that a file exists; whether it is the right file is a question about the lens, and nothing
downstream will notice if it is wrong. `bearing_node` back-projects pixels through it into a unit bearing, so a wrong
calibration yields bearings that are smooth, in range and consistently off. Nor can the simulator catch it:
[Simulation](#simulation) publishes bearings directly and never opens a calibration file.

So the check needs the real camera and a known geometry — the static-hold test. Put a blinking target at a measured
position relative to the camera, hold it there, and compare the fused output against the measurement:

- the fused `position` direction should match the direction to the target once composed through that camera's mount
  (`ros2 run tf2_tools view_frames` gives you the tree to compose with), and
- its magnitude should match the UWB range, which it does by construction — so the direction is the informative half.

A target straight ahead at a known distance is the cheapest first case, since a correct calibration must put it on the
optical axis and the residual is then the mount's error rather than the lens's. Repeating at the edge of the frame is
what actually distinguishes two calibration files whose coefficients differ. What counts as close enough depends on
the mount tolerance, so it is worth doing once per camera and recording the residual, rather than setting a threshold
here that this package cannot validate.

The mounts in `config/cameras.yaml` are **placeholders** — the file says so above the `cameras:` block. The
`yaw: π` on `camera_back` is the one structural value, since "facing aft" follows from the slot name, and the shared
`roll: -π/2` is what stands between the camera's optical axes and the body's — worth confirming against how the camera
is physically held, because every fused direction depends on it. The offsets and the ±0.349 rad splay on the side
cameras are guesses. `ros2 run tf2_tools view_frames` after a start, and compare against a tape measure.

### No hardware attached

Which is the state most development happens in, and it looks like a broken bringup while being correct: the camera
processes fail to open their devices and publish nothing, so detector, tracker and bearing come up and stay silent, and
each fusion node logs `Never received a UVDAR bearing for id ...` at `diagnostics_rate_hz`. All of them staying up is
the pass condition. For a run that produces data, see [Simulation](#simulation).

## Simulation

[sim_fusion.launch.py](launch/sim_fusion.launch.py) stands two publishers in for the two sensors and starts a fusion
node beside them, so the whole path to a fused position can be exercised with nothing attached:

```bash
ros2 launch mrs_ultraloc sim_fusion.launch.py
ros2 topic echo /uav/uwb_uvdar_fusion/targets      # default uav_name is uav, not $UAV_NAME
```

The positions are synthetic but arithmetically consistent — `x ≈ range × bearing_x` on the output is the check that the
pairing and the frame survived the launch. `--show-args` covers the addresses, the signal id, the trajectory geometry
and the rates.

`camera:=<slot>` picks which slot's bearing topic it feeds, and stamps bearings with the frame that slot would use, so a
simulated camera is indistinguishable from a real one to the node downstream. Run it against a live rig with one camera
looking at nothing — and give it the rig's `uav_name`, since this file defaults to `uav` rather than following
`$UAV_NAME`:

```bash
ros2 launch mrs_ultraloc two_cams.launch.py uav_name:=uav13 &
ros2 launch mrs_ultraloc sim_fusion.launch.py uav_name:=uav13 camera:=camera_front
```

The two names have to agree. A mismatch publishes bearings into a namespace the fusion node is not listening to, which
is not an error and looks exactly like a target that does not exist.

That is the way to test one camera of a multi-camera rig without the hardware, and it exercises the rig's own fusion
node rather than a second one. A second instance of the *launch file* is not the second-camera test — its three nodes
keep the vehicle namespace, so two instances collide on node names and double-publish one targets topic. For a second
camera, drive the simulator directly, which reaches the fusion node the rig already started:

```bash
ros2 run mrs_ultraloc sim_uvdar_target.py --ros-args \
    -p topic:=/uav13/uvdar/bearing/camera_back/observations \
    -p camera_frame:=uav13/camera_back -p signal_id:=0
```

To see the whole bringup — camera processes included, no hardware — start a rig launch file and read what it reports;
`dry_run:=true` goes as far as resolving and printing everything without starting any of it.

## Output frame

The position is published in the frame the bearing was stamped with, unrotated. Each camera's mount is published as a
TF static transform by the rig bringup (`<uav>/fcu` → `<uav>/<slot>`), so the transform is there for anyone who needs
it, and converting a fused position into the body frame is a `tf2` lookup rather than a change to this node.

What that does *not* settle is the multi-camera case. With N cameras the fusion happens N times and each result lives in
its own camera frame, so combining them means choosing which camera a given range should have been fused against —
and a UWB range says nothing about which camera is looking at the vehicle that owns it. That is a decision, not a
transform, and this package deliberately does not make it on a later stage's behalf. Per-slot frames and per-slot
output topics are what a combination stage needs to make it; see the note in
[launch/camera_rig.launch.py](launch/camera_rig.launch.py).

A mount that is wrong rather than merely unapplied is worse than an absent one, because it is silent: TF resolves, the
targets move plausibly, and they are simply in the wrong place. The mounts shipped in `config/cameras.yaml` are
placeholders — check them against a measurement before trusting a fused position's direction.
