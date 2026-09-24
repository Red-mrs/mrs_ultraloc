"""The one trajectory the two simulator nodes share, plus the two covariance helpers.

Why this is a module and not two copies
---------------------------------------
The fusion node computes `position = normalize(bearing) * distance` from two messages
that arrived at different times from different producers. If the two simulators
generated their own paths, then at the instant the fusion happens to pair a bearing
with a range those two numbers would describe different places, and every downstream
consumer would see a target whose position is inconsistent with its own reported
distance - which is indistinguishable from a bug in the fusion.

Both nodes therefore call `position_at(t)` on a trajectory built from the same
parameters. Same function, same inputs, same answer, so the pair the fusion combines
is self-consistent no matter how the two publish rates fall relative to each other.
"""

import math
import random

# The frame the bearing endpoint publishes in, and the frame a real Bluefox optical
# frame is named after. Names matter here only so a wrong frame_id in the fusion's
# output is obvious rather than plausible.
OPTICAL_FRAME = 'camera_0_optical_frame'


def unit_bearing(position):
    """Normalise a position into a unit bearing, or None if it is at the sensor.

    The fusion normalises the bearing itself, so a non-unit vector published here
    would still produce the right direction - but publishing what a real bearing
    endpoint publishes (a unit vector) means a consumer that trusts the length is
    testing something real.
    """
    norm = math.sqrt(sum(component * component for component in position))
    if norm <= 0.0:
        return None
    return tuple(component / norm for component in position)


def tangent_covariance(bearing, sigma_rad):
    """Rank-2 covariance of a unit bearing: sigma^2 over the plane normal to it.

    The shape the real bearing endpoint produces, and the shape the fusion node
    falls back to internally when a message carries none. Sending all-zeroes instead
    is not the same thing: the node reads an all-zero covariance as "no covariance"
    and substitutes its own `bearing_sigma_rad`, so a simulator that leaves it empty
    silently stops controlling the angular uncertainty it publishes.
    """
    s = sigma_rad ** 2
    return [[s * ((1.0 if i == j else 0.0) - bearing[i] * bearing[j]) for j in range(3)] for i in range(3)]


def flatten(matrix):
    return [matrix[i][j] for i in range(3) for j in range(3)]




class SmoothWiggle:
    """A bounded, deterministic, smooth wiggle of standard deviation `sigma`.

    Deterministic in t rather than drawn from a generator per sample, for the same
    reason the trajectory is: the two nodes run in separate processes and publish at
    different rates, so asking about an instant has to give the same answer however
    often and wherever it is asked. Random draws at publish time would break exactly
    the cross-node consistency this module exists to provide, and would make a failing
    run impossible to reproduce.

    A sum of harmonics of `period_sec`, so the result is smooth enough that a
    differentiated version is not dominated by whichever node sampled it, and periodic
    with the orbit so the simulation never wanders into a shape it does not repeat.

    `sigma` is the RMS of the output, not the amplitude of one component: the harmonic
    weights are drawn from a unit normal and then rescaled, so a caller can say "half a
    metre of multipath" and get that.
    """

    def __init__(self, sigma, period_sec, seed=0, harmonics=4):
        self.sigma = float(sigma)
        self._terms = []
        if self.sigma > 0.0:
            rng = random.Random(seed)
            weights = [rng.gauss(0.0, 1.0) for _ in range(harmonics)]
            norm = math.sqrt(sum(weight * weight for weight in weights) / 2.0) or 1.0
            for k, weight in enumerate(weights, start=1):
                self._terms.append((
                    self.sigma * weight / norm,
                    2.0 * math.pi * k / period_sec,
                    rng.uniform(0.0, 2.0 * math.pi),
                ))

    def __call__(self, t):
        return sum(amplitude * math.sin(frequency * t + phase)
                   for amplitude, frequency, phase in self._terms)


class SquareOrbit:
    """A target flying a rounded square in the x-z plane of an optical frame.

    Frame follows REP 103 for the optical frame the bearing endpoint publishes in:
    +x forward along the optical axis, +y left, +z up. So `centre_x` is depth, which
    is the axis the range lives on, and the lateral offset is `lateral_y`.

    The path is a square of half-side `half_side` centred at (`centre_x`, `lateral_y`,
    0), with the corners rounded over `corner_radius`. Rounding is not cosmetic:
    a sharp corner reverses two velocity components in one sample period, which
    produces an angular jump on the bearing that no real tracker would deliver. A
    radius of zero is the exact square, and a radius equal to the half-side is
    exactly a circle; the default is a quarter of the side, which reads as a square
    with the corners taken off.

    The closest approach is `hypot(centre_x - half_side, lateral_y)`, on the near
    straight. The farthest point is on the far corners, rounded inwards by the radius,
    so `range_bounds()` reports both from the path itself rather than from the corner
    formula. With the defaults that is about 1.1 m to 5.0 m.
    """

    def __init__(self, centre_x=3.0, lateral_y=0.5, half_side=2.0, corner_radius=None,
                 period_sec=20.0, noise_sigma_m=0.0, noise_seed=0):
        if centre_x <= 0.0:
            raise ValueError(f'centre_x must be in front of the camera, got {centre_x}')
        if half_side <= 0.0:
            raise ValueError(f'half_side must be positive, got {half_side}')
        if period_sec <= 0.0:
            raise ValueError(f'period_sec must be positive, got {period_sec}')

        self.centre_x = float(centre_x)
        self.lateral_y = float(lateral_y)
        self.half_side = float(half_side)
        self.period_sec = float(period_sec)
        self.noise_seed = int(noise_seed)
        # Clamped rather than rejected: a radius past the half-side just means the
        # straights have vanished, which is a circle, which is a fine thing to fly.
        self.corner_radius = float(half_side / 4.0 if corner_radius is None
                                   else min(max(corner_radius, 0.0), half_side))

        # One noise process per axis, offset in seed so the three are uncorrelated.
        self._noise = tuple(
            SmoothWiggle(noise_sigma_m, period_sec, seed=noise_seed + axis) for axis in range(3))

        self._segments = self._build_segments()
        self.perimeter = sum(segment[-1] for segment in self._segments)

    # ---- path construction ---------------------------------------------------

    def _build_segments(self):
        """Quarter arcs and straights in traversal order, each as (..., length) last.

        Traversal is counter-clockwise in the x-z plane plotted with x right and z up,
        starting at the bottom of the path.
        """
        a = self.half_side
        r = self.corner_radius
        x0 = self.centre_x

        # Corner, incoming direction, outgoing direction. Counter-clockwise, so every
        # turn is +90 degrees and every arc sweeps exactly +pi/2.
        corners = [
            ((x0 + a, -a), (1.0, 0.0), (0.0, 1.0)),
            ((x0 + a, +a), (0.0, 1.0), (-1.0, 0.0)),
            ((x0 - a, +a), (-1.0, 0.0), (0.0, -1.0)),
            ((x0 - a, -a), (0.0, -1.0), (1.0, 0.0)),
        ]

        def arc_start(corner):
            # Where that corner's arc takes over from the incoming straight.
            (cx, cz), d_in, _ = corner
            return (cx - r * d_in[0], cz - r * d_in[1])

        pieces = []
        for index, ((cx, cz), d_in, d_out) in enumerate(corners):
            # The arc is tangent to both straights, so its centre sits one radius
            # inside each of them. Inside is the incoming direction turned a quarter
            # turn counter-clockwise, n = (-d_z, d_x), for both edges at once.
            n_in = (-d_in[1], d_in[0])
            n_out = (-d_out[1], d_out[0])
            centre = (cx + r * n_in[0] + r * n_out[0], cz + r * n_in[1] + r * n_out[1])
            start = arc_start(corners[index])
            end = (cx + r * d_out[0], cz + r * d_out[1])   # where it hands back
            start_angle = math.atan2(start[1] - centre[1], start[0] - centre[0])
            pieces.append(('arc', centre, r, start_angle, math.pi / 2.0, r * math.pi / 2.0))

            # Then straight until the next corner's arc begins. Zero-length when the
            # radius has eaten the whole side, which is the circle case.
            next_start = arc_start(corners[(index + 1) % len(corners)])
            dx, dz = next_start[0] - end[0], next_start[1] - end[1]
            length = math.hypot(dx, dz)
            if length > 1e-12:
                pieces.append(('line', end, (dx / length, dz / length), length))

        return pieces

    def _position_along(self, s):
        """(x, z) at arc length `s` from the start, with s already inside one period."""
        for kind, *rest in self._segments:
            length = rest[-1]
            if s <= length:
                if kind == 'line':
                    origin, direction, _ = rest
                    return (origin[0] + direction[0] * s, origin[1] + direction[1] * s)
                centre, radius, start_angle, sweep, _ = rest
                angle = start_angle + sweep * (s / length if length else 0.0)
                return (centre[0] + radius * math.cos(angle),
                        centre[1] + radius * math.sin(angle))
            s -= length

        # Reachable only when the wrapped arc length lands a rounding hair past the
        # perimeter, or when the radius ate a whole side, that segment was dropped as
        # zero-length, and the remainder slipped over the end. Both mean "as far round
        # as the path goes", which is the end of the last segment - and since the path
        # is closed, that point is also its start, so this returns a place on the loop
        # instead of jumping.
        kind, *rest = self._segments[-1]
        if kind == 'line':
            origin, direction, length = rest
            return (origin[0] + direction[0] * length, origin[1] + direction[1] * length)
        centre, radius, start_angle, sweep, _ = rest
        angle = start_angle + sweep
        return (centre[0] + radius * math.cos(angle), centre[1] + radius * math.sin(angle))

    def _position_in_plane(self, t):
        """(x, z) at time t, at constant speed along the rounded square."""
        return self._position_along((t / self.period_sec) % 1.0 * self.perimeter)

    # ---- public interface ----------------------------------------------------

    def position_at(self, t):
        """Target position (x, y, z) in the optical frame at time t seconds.

        Pure function of t, which is what lets two independent processes agree.
        """
        x, z = self._position_in_plane(t)
        # Signed: this is the optical frame, so negative z is genuinely below the
        # camera rather than a second copy of above, and the fusion node preserves
        # the sign (a legacy ROS 1 fusion computed z as sqrt(d^2 - x^2 - y^2) and
        # could not represent below-axis targets at all).
        return (x + self._noise[0](t),
                self.lateral_y + self._noise[1](t),
                z + self._noise[2](t))

    def distance_at(self, t):
        x, y, z = self.position_at(t)
        return math.sqrt(x * x + y * y + z * z)

    def bearing_at(self, t):
        """Unit bearing at t, or None if the target is at the sensor.

        Unreachable with any sane parameters, but a degenerate set must not divide by
        zero at publish time and take the node down mid-run.
        """
        return unit_bearing(self.position_at(t))

    def range_bounds(self, samples=720):
        """(nearest, farthest) distance the *un-noised* path reaches, for logging.

        Sampled rather than derived: the farthest point is on a rounded corner, so a
        closed form has to know whether the ray from the sensor through that arc's
        centre actually falls inside the arc, and getting that subtly wrong would make
        the log claim a distance range the sim never visits. The noise is excluded
        because it is a wiggle around the path, not part of it - with noise on, expect
        to go a little outside what this reports.
        """
        distances = [math.hypot(*self._position_in_plane(t)) for t in
                     (self.period_sec * i / samples for i in range(samples))]
        # hypot of three, not two - the lateral offset is constant, so it lifts every
        # distance by the same quadrature amount.
        lateral = self.lateral_y
        return (math.sqrt(min(distances) ** 2 + lateral * lateral),
                math.sqrt(max(distances) ** 2 + lateral * lateral))
