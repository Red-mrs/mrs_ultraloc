"""Pins the DDS middleware for the whole test session.

This has to happen at module import time, in a conftest, and not in a fixture.

Importing a generated message package dlopen()s its typesupport, and ROS resolves the
RMW at that point - so by the time a fixture body runs, the middleware is already
chosen and writing os.environ afterwards changes nothing. The failure is the worst
kind: the node starts and logs its happy configuration line, the test process quietly
initialises rmw_zenoh_cpp (the default here, which needs a router and otherwise
discovers nobody), and every test times out waiting for a topic that is visible by
name and has no discoverable publisher.

pytest imports conftest.py before any test module in the directory, so assigning here
runs ahead of `import fusion_test_helpers` and the message types it pulls in.

Run against a different middleware or domain with:

    FUSION_TEST_RMW=rmw_zenoh_cpp FUSION_TEST_DOMAIN_ID=7 pytest test/test_fusion.py
"""

import os

DEFAULT_RMW = 'rmw_fastrtps_cpp'
DEFAULT_DOMAIN_ID = '77'

# Assigned, not setdefault: RMW_IMPLEMENTATION is already exported as rmw_zenoh_cpp by
# the ROS setup files, so setdefault would leave the default in place while reading as
# though the middleware had been pinned.
os.environ['RMW_IMPLEMENTATION'] = os.environ.get('FUSION_TEST_RMW', DEFAULT_RMW)

# A domain of its own, so a real robot's bearing and UWB traffic on this machine can
# neither satisfy nor spoil an assertion.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('FUSION_TEST_DOMAIN_ID', DEFAULT_DOMAIN_ID)
