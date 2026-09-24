"""Offline simulators for the UWB + UVDAR fusion input side.

Only `sim_trajectory` lives here; the two nodes stay in `scripts/` so the launch file
can name them with `package=`/`executable=`, which resolves against the executables
installed under `lib/mrs_ultraloc` rather than against a Python entry point.

Both nodes import this module, and neither is importable as a module itself -
`ament_index_python` puts the installed copy of this package on sys.path from
`share/mrs_ultraloc/python`, which the script wrappers do before importing.
"""
