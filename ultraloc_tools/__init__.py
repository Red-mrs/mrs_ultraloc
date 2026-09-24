"""Shared Python helpers for this package's Python nodes.

Two modules, for two kinds of sharing:

* `parameters` - parameter declaration and coercion, used by every Python node here.
  It is not sim-only: the ROS parameter type rules apply to a visualizer exactly as
  much as to a simulator.
* `sim_trajectory` - the one path the two simulator nodes agree on. Used by the
  simulators alone; a node that only reads the fusion's output has no business
  rebuilding the ground truth.

Installed via `ament_python_install_package()`, which is what registers the package's
Python directory with the environment hooks so `import ultraloc_tools` works in a
sourced shell. The two simulator scripts still carry a short sys.path fallback for
being run straight out of a checkout.
"""
