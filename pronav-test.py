"""Proportional-navigation (PN) guidance simulator.

A pursuer chases a target in a 2D plane. At each timestep the pursuer measures
the *line-of-sight (LOS) angle* to the target and turns its velocity vector in
proportion to how fast that angle is changing:

    flight_path_angle += N * (LOS_angle_change)

`N` is the navigation gain. The key idea of PN: if the LOS angle stops changing
(constant bearing), the two bodies are on a collision course. A higher `N` drives
the LOS rate to zero faster, producing an intercept.

The public entry point is :func:`simulate`, which takes the gain and every
initial condition as keyword arguments and returns per-step trajectory histories.
It is designed to be imported and driven from a notebook or other scripts.
"""

import math


def angle_difference(a_deg, b_deg):
    """Smallest signed difference ``a - b`` wrapped to [-180, 180) degrees.

    atan2 returns angles on a -180..180 branch, so a raw subtraction jumps by
    ~360 deg when the line of sight crosses that branch cut. Wrapping the
    difference keeps the line-of-sight *rate* continuous across all quadrants.
    """
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


class Entity:
    """A point mass in 2D with a position and velocity."""

    def __init__(self, pos_x, pos_y, vel_x, vel_y):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.vel_x = vel_x
        self.vel_y = vel_y

    def step(self):
        """Advance the position by one timestep using the current velocity."""
        self.pos_x += self.vel_x
        self.pos_y += self.vel_y


class Target(Entity):
    """The pursued body. Moves in a straight line at constant velocity."""


class Pursuer(Entity):
    """The chaser. Steers via proportional navigation toward the target."""

    def __init__(self, pos_x, pos_y, vel_x, vel_y, flight_path_angle):
        super().__init__(pos_x, pos_y, vel_x, vel_y)
        self.flight_path_angle = flight_path_angle

    def rotate_velocity(self, delta_fpa_deg):
        """Rotate the velocity vector by delta_fpa (degrees), preserving speed.

        A positive delta_fpa turns the velocity one way (clockwise) and a
        negative one turns it the other (counter-clockwise), so the pursuer
        reverses its turn automatically whenever the guidance command changes
        sign -- which is what lets it overshoot and swing back.

        Both new components are computed from the *original* vx/vy (snapshotted
        below). Using the freshly-overwritten vx -- as an earlier version did --
        is not a rotation: it neither preserves speed nor reverses correctly.
        """
        delta_rad = delta_fpa_deg * math.pi / 180.0
        cos_d = math.cos(delta_rad)
        sin_d = math.sin(delta_rad)
        vx, vy = self.vel_x, self.vel_y
        self.vel_x = vx * cos_d + vy * sin_d
        self.vel_y = -vx * sin_d + vy * cos_d


def simulate(
    N=1,
    steps=10,
    target_pos=(0, 0),
    target_vel=(-10, 0),
    pursuer_pos=(0, 500),
    pursuer_vel=(0, -15),
    pursuer_fpa=-102,
    initial_losa=90,
):
    """Run the proportional-navigation sim with the given gain and initial
    conditions.

    Returns a dict of per-step histories (lists, one entry per recorded step):
    ``losa``, ``fpa`` (degrees) and the ``target_x/target_y`` /
    ``pursuer_x/pursuer_y`` positions, for plotting trajectories.
    """
    target = Target(target_pos[0], target_pos[1], target_vel[0], target_vel[1])
    pursuer = Pursuer(
        pursuer_pos[0], pursuer_pos[1], pursuer_vel[0], pursuer_vel[1], pursuer_fpa
    )

    line_of_sight_angle = initial_losa
    prev_line_of_sight_angle = line_of_sight_angle
    prev_flight_path_angle = pursuer.flight_path_angle

    history = {
        "losa": [],
        "fpa": [],
        "target_x": [],
        "target_y": [],
        "pursuer_x": [],
        "pursuer_y": [],
    }

    for index in range(steps):
        # 1. Advance both bodies one step using their current velocities.
        target.step()
        pursuer.step()

        # 2. Measure the line-of-sight angle of the target relative to the
        #    pursuer. Signed differences feed atan2, so the angle is correct in
        #    all four quadrants (range -180..180 deg).
        range_x = pursuer.pos_x - target.pos_x
        range_y = pursuer.pos_y - target.pos_y
        line_of_sight_angle_rad = math.atan2(range_y, range_x)
        line_of_sight_angle = line_of_sight_angle_rad * 180.0 / math.pi

        # 3. Proportional-navigation law: change the flight-path angle in
        #    proportion (gain N) to the change in the LOS angle. The LOS rate is
        #    wrapped so it stays continuous across the +/-180 deg branch.
        los_rate = angle_difference(line_of_sight_angle, prev_line_of_sight_angle)
        pursuer.flight_path_angle = N * los_rate + prev_flight_path_angle

        # 4. Record this step's state for plotting / inspection.
        history["losa"].append(line_of_sight_angle)
        history["fpa"].append(pursuer.flight_path_angle)
        history["target_x"].append(target.pos_x)
        history["target_y"].append(target.pos_y)
        history["pursuer_x"].append(pursuer.pos_x)
        history["pursuer_y"].append(pursuer.pos_y)

        # 5. Carry state to the next step; delta_fpa is how much to turn by.
        prev_line_of_sight_angle = line_of_sight_angle
        delta_fpa = prev_flight_path_angle - pursuer.flight_path_angle
        prev_flight_path_angle = pursuer.flight_path_angle

        # 6. Steer the pursuer by rotating its velocity vector by delta_fpa.
        #    The turn takes effect on the next iteration's step (1).
        pursuer.rotate_velocity(delta_fpa)

    return history


def run_simulation(N, steps=10):
    """Run the sim with the default initial conditions and return only the
    line-of-sight angle (degrees) recorded at each step."""
    return simulate(N=N, steps=steps)["losa"]


if __name__ == "__main__":
    for index, losa in enumerate(run_simulation(N=1)):
        print(index, losa)
