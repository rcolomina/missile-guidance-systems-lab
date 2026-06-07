# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A small numerical lab exploring **proportional navigation (PN)**, the classic
guidance law for pursuit/intercept problems. A pursuer chases a target in a 2D
plane; the simulation steps a kinematic model forward in discrete time and tracks
the line-of-sight angle (LOS) and flight-path angle (FPA) of the pursuer.

Exploratory math/control code, not a packaged application. The simulator itself
uses only the standard library (`math`); the notebook and tests add `matplotlib`,
`ipywidgets`, `numpy`, and `pytest` (all already installed in this environment).

## Files

- `pronav-test.py` — the simulator (classes + `simulate()` / `run_simulation()`).
- `test_pronav.py` — pytest checks on stabilization behavior vs gain `N`.
- `pronav_playground.ipynb` — interactive notebook with sliders for `N` and all
  initial conditions; plots trajectories and angle-vs-step.

## Commands

```bash
python3 pronav-test.py          # run the default scenario, prints index LOSA per step
python3 -m pytest test_pronav.py -q   # run the tests
python3 -m pytest test_pronav.py::test_n5_stabilizes -q   # a single test
jupyter lab pronav_playground.ipynb   # open the playground
```

## Architecture

`pronav-test.py` defines `Entity` (a 2D point mass with `pos_*`/`vel_*` and a
`step()` integrator), subclassed by `Target` (constant velocity) and `Pursuer`
(adds `flight_path_angle` and `rotate_velocity()`).

The simulation entry point is **`simulate(N, steps, target_pos, target_vel,
pursuer_pos, pursuer_vel, pursuer_fpa, initial_losa)`** — every initial condition
is a keyword argument with a default matching the original scenario. It returns a
dict of per-step lists: `losa`, `fpa`, `target_x/target_y`, `pursuer_x/pursuer_y`.
This is what the notebook drives. `run_simulation(N, steps)` is a thin wrapper that
returns just the `losa` list with default initial conditions (used by the tests and
the `__main__` print loop).

The PN law applied each step:

```
flight_path_angle = N * angle_difference(LOS[i], LOS[i-1]) + flight_path_angle[i-1]
```

`N` is the navigation gain. Each step the loop advances both bodies once, measures
the LOS angle (signed `atan2`, valid in all four quadrants), applies the PN law,
records state, then rotates the pursuer's velocity by `delta_fpa` (the turn takes
effect on the next step). The physics: when the LOS rate drives toward zero
(constant bearing), the bodies are on a collision course; higher `N` flattens the
LOS faster. The tests assert this — N=1 diverges (LOS rate grows), N=5 converges
(LOS rate shrinks) — and verify it holds from every cardinal approach.

`Pursuer.rotate_velocity()` is a true speed-preserving 2D rotation: the sign of
`delta_fpa` sets the turn direction (clockwise vs counter-clockwise), so the
pursuer **reverses its turn automatically** when the guidance command changes sign
(e.g. overshooting and swinging back). This emerges from the PN law and the LOS
rate's sign — do **not** special-case it on axis crossings.

`angle_difference(a, b)` is a module-level helper returning the signed difference
wrapped to `[-180, 180)`. It is essential: `atan2` lives on a -180..180 branch, so
a raw LOS subtraction would jump ~360° when the line of sight crosses that cut
(e.g. a target approached from the `left`, LOS ≈ 180°). Use it for any LOS-rate math.

### Importing the simulator

The module filename has a hyphen (`pronav-test.py`), so it can't be imported with a
plain `import`. Both the tests and the notebook load it via
`importlib.import_module("pronav-test")`.

### Known rough edges to be aware of when editing

- **Mixed units.** Angles are tracked in degrees; `rotate_velocity()` converts to
  radians inline. Keep conversions explicit when changing the guidance math.
- **Near-singularity at zero miss distance.** When the pursuer passes almost
  exactly through the target, the LOS rate spikes toward ±180°/step and the discrete
  trajectory becomes erratic. This is the geometry, not a control bug — it's most
  visible with a (near-)stationary target and a large step size.
- **Changing the simulation math changes the pinned outputs.** Re-run the tests and
  re-execute the notebook after edits; update `test_default_scenario_regression`'s
  pinned values if the core dynamics legitimately change.

When changing the simulation math, re-run the tests and re-execute the notebook
(`jupyter nbconvert --to notebook --execute --inplace pronav_playground.ipynb`) so
its saved outputs stay consistent.
