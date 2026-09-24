"""The ROS-side plumbing both simulator nodes share.

Two things live here: `TRAJECTORY_PARAMETERS` with its declare/read helpers, which is
one definition of the path parameters so the launch file cannot hand the two nodes two
different paths; and the declare/read pairs, which exist because ROS parameter types are
exact.

(The import bootstrap that makes `sim_ultraloc` importable from an installed script is
deliberately *not* here - it cannot be, since it has to run before this module can be
imported at all. It is a few lines at the top of each script instead.)

A parameter declared with a float default rejects an integer override:

    ros2 run mrs_ultraloc sim_uwb_module.py --ros-args -p publish_rate_hz:=42
    InvalidParameterTypeException: Trying to set parameter 'publish_rate_hz' to '42'
    of type 'INTEGER', expecting type 'DOUBLE'

`42` is what a person types and `42.0` is what the default is, so the natural way to
change the rate would crash the node at startup with a message about types. Declaring
dynamic and coercing on read accepts `42`, `42.0` and `42e0` alike.

The same kind of divergence bites harder elsewhere in this stack, where it is not
fixable from here: launch writes temporary parameter files with PyYAML and the node reads
them with yaml-cpp, and the two disagree about whether a token like `2075E33814` is a
float. A UWB serial of that shape reaches the driver as a number and aborts it. Nothing
simulated here has that shape - these are lengths, rates and addresses - so coercing on
read is the whole fix rather than a workaround for a second problem.
"""

import math

from rcl_interfaces.msg import ParameterDescriptor


# ---------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------

def _dynamic():
    """A descriptor saying "the type here is not fixed".

    `dynamic_typing` is a field of ParameterDescriptor, not a keyword argument of
    `declare_parameter` - passing it as one is a TypeError on the first parameter the
    node declares, which is a cheap mistake to make and an odd-looking one to read.
    """
    return ParameterDescriptor(dynamic_typing=True)


def declare_float(node, name, default):
    """A double parameter that also accepts an integer override. See the module docstring."""
    node.declare_parameter(name, float(default), descriptor=_dynamic())


def read_float(node, name):
    value = node.get_parameter(name).value
    if value is None:
        raise ValueError(f'parameter \'{name}\' is unset and has no default')
    if isinstance(value, bool):
        raise ValueError(f'parameter \'{name}\' must be a number, got {value!r}')
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'parameter \'{name}\' must be a number, got {value!r}') from exc


def declare_int(node, name, default):
    """An integer parameter that also accepts a float override."""
    node.declare_parameter(name, int(default), descriptor=_dynamic())


def read_int(node, name):
    """Read back a `declare_int` parameter as an int.

    Rounded rather than truncated: `noise_seed:=3.7` means 4 to whoever typed it, and
    silently becoming 3 would pick a different path out of the same generator, which is
    the one property of that parameter a person relies on.
    """
    value = node.get_parameter(name).value
    if value is None:
        raise ValueError(f'parameter \'{name}\' is unset and has no default')
    if isinstance(value, bool):
        # bool is an int in Python, and `true` is not a seed anyone means.
        raise ValueError(f'parameter \'{name}\' must be a number, got {value!r}')
    try:
        return int(round(float(value)))
    except (TypeError, ValueError) as exc:
        raise ValueError(f'parameter \'{name}\' must be a whole number, got {value!r}') from exc


def declare_string(node, name, default):
    """A string parameter. Strings are exact in ROS, so only the coercion is missing."""
    node.declare_parameter(name, str(default), descriptor=_dynamic())


def read_string(node, name):
    value = node.get_parameter(name).value
    if not isinstance(value, str):
        # A frame_id of 0 is not a frame; letting it through hands the consumer a header
        # it rejects a long way from here.
        raise ValueError(f'parameter \'{name}\' must be a string, got {value!r}')
    return value


def read_rate(node, name):
    """A positive rate, checked at startup rather than at the first timer callback.

    `create_timer` with a non-positive period raises a confusing rcl error from inside
    the timer implementation; better to say which parameter is wrong.
    """
    rate_hz = read_float(node, name)
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError(f'parameter \'{name}\' must be a positive rate in Hz, got {rate_hz}')
    return rate_hz


# ---------------------------------------------------------------------------
# the shared trajectory
# ---------------------------------------------------------------------------

#: Sentinel for `corner_radius` meaning "let SquareOrbit choose".
#:
#: A negative radius is meaningless, so it can carry "unset" without stopping
#: `corner_radius:=0` from meaning a genuine sharp corner. Declaring the parameter
#: without a value instead - by type - would be more honest, but reading an unset
#: parameter raises in rclpy, so a sentinel is what fits.
CORNER_RADIUS_AUTO = -1.0

#: Every `SquareOrbit` keyword and the default the nodes declare for it.
#:
#: Both nodes declare these as ROS parameters and the launch file gives them the *same*
#: values, so the defaults live here rather than being typed twice: if they drifted
#: apart, each node would fly a self-consistent path and the two would agree on nothing,
#: which looks exactly like a fusion bug and is not one.
TRAJECTORY_PARAMETERS = {
    'centre_x': 3.0,
    'lateral_y': 0.5,
    'half_side': 2.0,
    'corner_radius': CORNER_RADIUS_AUTO,
    'period_sec': 20.0,
    'noise_sigma_m': 0.0,
    'noise_seed': 0,
}


def declare_trajectory_parameters(node, prefix='trajectory_'):
    """Declare one ROS parameter per `SquareOrbit` keyword, under `prefix`.

    Prefixed because the UWB node has a noise of its own as well, and an unprefixed
    `noise_sigma_m` would mean two different things in one node.
    """
    for name, default in TRAJECTORY_PARAMETERS.items():
        if name == 'noise_seed':
            declare_int(node, prefix + name, default)
        else:
            declare_float(node, prefix + name, default)


def trajectory_from_parameters(node, prefix='trajectory_'):
    """Build the path from what `declare_trajectory_parameters` declared."""
    from sim_ultraloc.sim_trajectory import SquareOrbit

    kwargs = {name: (read_int(node, prefix + name) if name == 'noise_seed'
                     else read_float(node, prefix + name))
              for name in TRAJECTORY_PARAMETERS}

    if kwargs['corner_radius'] < 0.0:
        kwargs['corner_radius'] = None

    return SquareOrbit(**kwargs)
