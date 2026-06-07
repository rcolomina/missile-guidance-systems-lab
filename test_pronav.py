"""Tests for the proportional-navigation simulator.

Covers three things:
  * the stabilization behavior vs gain N (N=1 diverges, N=5 converges),
  * the multi-quadrant line-of-sight geometry (signed atan2 + angle wrapping),
  * a regression pin on the default scenario so the core logic can't drift silently.
"""

import math
import importlib

pronav = importlib.import_module("pronav-test")
simulate = pronav.simulate
angle_difference = pronav.angle_difference
Pursuer = pronav.Pursuer


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def los_rates(losa_history):
    """Per-step magnitude of the LOS angle change (deg/step), wrap-aware."""
    return [
        abs(angle_difference(losa_history[i + 1], losa_history[i]))
        for i in range(len(losa_history) - 1)
    ]


def is_stabilizing(losa_history):
    """True if the LOS rate shrinks from start to end of the run (converging)."""
    rates = los_rates(losa_history)
    return rates[-1] < rates[0]


def los0(pursuer_pos, target_pos):
    """True initial line-of-sight angle (deg) of the geometry, before any step."""
    return math.degrees(
        math.atan2(pursuer_pos[1] - target_pos[1], pursuer_pos[0] - target_pos[0])
    )


# Four intercept scenarios, one per cardinal approach, spanning all quadrants and
# the +/-180 deg branch cut. Each is the default engagement rotated 90 deg.
# (pursuer_fpa is irrelevant to the trajectory -- only its per-step delta steers.)
CARDINAL_SCENARIOS = {
    "above": dict(target_pos=(0, 0), target_vel=(-10, 0), pursuer_pos=(0, 500), pursuer_vel=(0, -15)),
    "below": dict(target_pos=(0, 0), target_vel=(10, 0), pursuer_pos=(0, -500), pursuer_vel=(0, 15)),
    "right": dict(target_pos=(0, 0), target_vel=(0, -10), pursuer_pos=(500, 0), pursuer_vel=(-15, 0)),
    "left": dict(target_pos=(0, 0), target_vel=(0, 10), pursuer_pos=(-500, 0), pursuer_vel=(15, 0)),
}


def run_scenario(name, N, steps=10):
    s = CARDINAL_SCENARIOS[name]
    return simulate(
        N=N, steps=steps, pursuer_fpa=0.0,
        initial_losa=los0(s["pursuer_pos"], s["target_pos"]), **s,
    )


# --------------------------------------------------------------------------- #
# angle_difference (wrap handling that makes multi-quadrant work)
# --------------------------------------------------------------------------- #
def test_angle_difference_no_wrap():
    assert angle_difference(88.8, 90.0) == -1.2 or abs(angle_difference(88.8, 90.0) + 1.2) < 1e-9
    assert abs(angle_difference(10.0, 0.0) - 10.0) < 1e-9


def test_angle_difference_wraps_across_branch_cut():
    # Crossing +/-180 must give the short way round, not a ~360 deg jump.
    assert abs(angle_difference(-179.0, 179.0) - 2.0) < 1e-9
    assert abs(angle_difference(179.0, -179.0) + 2.0) < 1e-9


def test_angle_difference_in_range():
    for a in range(-360, 361, 17):
        for b in range(-360, 361, 23):
            d = angle_difference(float(a), float(b))
            assert -180.0 <= d < 180.0


# --------------------------------------------------------------------------- #
# velocity rotation (must be a true rotation: preserves speed, reverses by sign)
# --------------------------------------------------------------------------- #
def test_rotation_preserves_speed():
    for delta in (-170, -90, -30, 0, 45, 135, 200):
        p = Pursuer(0, 0, 3.0, -4.0, 0)  # speed 5
        p.rotate_velocity(delta)
        assert abs(math.hypot(p.vel_x, p.vel_y) - 5.0) < 1e-9, f"delta={delta}"


def test_turn_direction_follows_command_sign():
    # Equal and opposite commands must rotate the velocity to mirror-image
    # headings -- i.e. one clockwise, the other counter-clockwise.
    pos = Pursuer(0, 0, 1.0, 0.0, 0)
    neg = Pursuer(0, 0, 1.0, 0.0, 0)
    pos.rotate_velocity(30)
    neg.rotate_velocity(-30)
    assert abs(pos.vel_x - neg.vel_x) < 1e-9       # same x component
    assert abs(pos.vel_y + neg.vel_y) < 1e-9       # opposite y component
    assert abs(pos.vel_y) > 1e-6                    # it actually turned


# --------------------------------------------------------------------------- #
# multi-quadrant line-of-sight geometry (signed atan2, not fabs)
# --------------------------------------------------------------------------- #
def test_los_angle_per_quadrant():
    # pursuer at origin, target placed so the pursuer->...->target vector
    # (pursuer - target) lands in each quadrant. fabs would collapse all to +45.
    cases = {
        (-10, -10): 45.0,    # vector (+10,+10) -> Q1
        (10, -10): 135.0,    # vector (-10,+10) -> Q2
        (10, 10): -135.0,    # vector (-10,-10) -> Q3
        (-10, 10): -45.0,    # vector (+10,-10) -> Q4
    }
    for (tx, ty), expected in cases.items():
        h = simulate(
            N=0, steps=1, target_pos=(tx, ty), target_vel=(0, 0),
            pursuer_pos=(0, 0), pursuer_vel=(0, 0), pursuer_fpa=0, initial_losa=0,
        )
        assert abs(h["losa"][0] - expected) < 1e-9, f"target {(tx, ty)} -> {h['losa'][0]}"


def test_los_stays_in_valid_range():
    for name in CARDINAL_SCENARIOS:
        for angle in run_scenario(name, N=5)["losa"]:
            assert -180.0 < angle <= 180.0


# --------------------------------------------------------------------------- #
# stabilization vs gain N
# --------------------------------------------------------------------------- #
def test_n1_does_not_stabilize():
    rates = los_rates(simulate(N=1, steps=10)["losa"])
    assert not (rates[-1] < rates[0]), (
        f"N=1 LOS rate should grow, went {rates[0]:.3f} -> {rates[-1]:.3f} deg/step"
    )


def test_n5_stabilizes():
    rates = los_rates(simulate(N=5, steps=10)["losa"])
    assert rates[-1] < rates[0], (
        f"N=5 LOS rate should shrink, went {rates[0]:.3f} -> {rates[-1]:.3f} deg/step"
    )


def test_higher_gain_stabilizes_more():
    assert los_rates(simulate(N=5)["losa"])[-1] < los_rates(simulate(N=1)["losa"])[-1]


def test_stabilization_holds_in_every_quadrant():
    # The N=1-diverges / N=5-converges contrast should hold from any approach,
    # including "left" which starts on the +/-180 branch cut (exercises wrapping).
    for name in CARDINAL_SCENARIOS:
        assert not is_stabilizing(run_scenario(name, N=1)["losa"]), f"{name} N=1 should diverge"
        assert is_stabilizing(run_scenario(name, N=5)["losa"]), f"{name} N=5 should converge"


# --------------------------------------------------------------------------- #
# regression pin (guards against accidental logic changes)
# --------------------------------------------------------------------------- #
def test_default_scenario_regression():
    expected = [
        88.81881108667336, 87.60099576453733, 86.34535466491607, 85.05069900129146,
        83.71586197216295, 82.33971214768377, 80.92116903576698, 79.45922101211956,
        77.95294577585697, 76.40153345491471,
    ]
    got = simulate(N=1, steps=10)["losa"]
    assert len(got) == len(expected)
    for g, e in zip(got, expected):
        assert abs(g - e) < 1e-9


if __name__ == "__main__":
    funcs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in funcs:
        fn()
        print("ok", fn.__name__)
    print(f"\nAll {len(funcs)} tests passed.")
