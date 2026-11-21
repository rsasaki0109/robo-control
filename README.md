# robo-control

Robot models, controllers and a deterministic simulator for comparing control
laws **per robot body**: wheeled, quadruped, humanoid, arm, drone and
cart-pole. Reference paths come from
[robo-planning](https://github.com/rsasaki0109/robo-planning) (planning by task).

機体（車輪型・犬型・ヒューマノイド・アーム・ドローン・倒立振子）ごとに、
合う制御則を同じタスクで比較シミュレーションするライブラリです。

## Gallery by robot body

### Wheeled

| Differential-drive AMR | Car |
|:---:|:---:|
| ![amr](docs/media/amr.gif) | ![car](docs/media/car.gif) |
| A* -> spline -> speed profile in a warehouse; **pure pursuit / Kanayama / LQR / LTV-MPC** on a unicycle with velocity-loop lag | Fast spline circuit (up to 12 m/s) on a kinematic bicycle with steering lag; **pure pursuit / Stanley / LQR / LTV-MPC** |

### Legged

| Quadruped (dog type) | Humanoid |
|:---:|:---:|
| ![quadruped](docs/media/quadruped.gif) | ![humanoid](docs/media/humanoid.gif) |
| Trot on a sagittal single-rigid-body model, speed steps and a 24 N s backward push; **PD + force-distribution QP vs convex MPC** over one gait cycle (3D versions run the articulated Go2, below) | 3D LIPM walking, 0.35 / 0.25 m/s push mid-step; **ZMP preview control on fixed footsteps falls, DCM control with step adjustment recovers** (the same walkers drive the articulated G1, below) |

### Manipulator, aerial, balancing

| Arm | Drone | Cart-pole |
|:---:|:---:|:---:|
| ![arm](docs/media/arm.gif) | ![drone](docs/media/drone.gif) | ![cartpole](docs/media/cartpole.gif) |
| RRT-Connect pick-and-place with a 2 kg payload the controllers do not know; **PD / PD + gravity / computed torque** | RRT* path through a forest with a wind gust; **cascaded PID / LQR / LTV-MPC** | Swing-up from hanging and a push; **energy shaping + LQR vs iLQR + TV-LQR** |

```bash
cargo run --release --example body_gallery              # all GIFs + metric tables -> out/bodies/
cargo run --release --example body_gallery humanoid     # one of: amr car quadruped humanoid arm drone cartpole
```

## On Robot Native Engine: articulated Unitree Go2 and G1

The same controllers also run on the full articulated robots of
[Robot Native Engine](https://github.com/rsasaki0109/RobotNativeEngine) (RNE),
in the `rne/` crate (`robo-control-rne`, RNE pinned to commit `8ff1cb4`):
RNE's hard-contact floating-base dynamics (`rne_dynamics::contact_step`) is
the plant, robo-control decides.

| Go2 (12 DoF) | G1 (12-DoF legs) |
|:---:|:---:|
| ![go2 on rne](docs/media/rne_go2.gif) | ![g1 on rne](docs/media/rne_g1.gif) |
| `legged3d::PdForceQp` vs `legged3d::ConvexMpc3d` choose the stance forces, applied as `-J^T f` on RNE's `Quadruped`; swing feet land at Raibert footholds through its leg IK. 0.5 then 1.0 m/s, a 120 N back push and a 100 N side push. | `scenarios::humanoid::LipmWalker` (ZMP preview on fixed steps vs DCM step adjustment, the same walker as the point-mass scenario) drives a force-QP whole-body controller on RNE's mass matrix, bias forces and Jacobians; four sole contacts per foot. |

```bash
cargo run --release -p robo-control-rne --example rne_gallery     # out/rne/{go2,g1}.gif + metrics
cargo test --release -p robo-control-rne                          # asserts the results below
```

#### Go2 on RNE

| controller | fell | rms_speed_err_mps | rms_height_err_m | rms_tilt_rad | lateral_drift_m | distance_m |
|---|---|---|---|---|---|---|
| PD + force QP | 0.000 | 0.252 | 0.005 | 0.113 | 0.282 | 3.468 |
| convex MPC | 0.000 | 0.143 | 0.009 | 0.055 | 0.143 | 4.473 |

#### G1 on RNE

| controller | fell | fall_time_s | rms_speed_err_mps | distance_m | lateral_drift_m |
|---|---|---|---|---|---|
| ZMP preview (fixed steps) (push) | 1.000 | 3.200 | 0.292 | 0.611 | 0.715 |
| DCM + step adjustment (push) | 0.000 | NaN | 0.085 | 2.294 | -0.003 |
| ZMP preview (fixed steps) (no push) | 1.000 | 3.200 | 0.054 | 0.591 | 0.703 |
| DCM + step adjustment (no push) | 0.000 | NaN | 0.060 | 2.194 | 0.083 |

What carried over and what did not:

- **Go2**: both force controllers trot the articulated robot unchanged. The
  convex MPC tracks speed better (0.14 vs 0.25 m/s RMS, including both pushes)
  and stays more level after the side push (3 vs 6.5 degrees RMS tilt).
- **G1**: DCM step adjustment walks 8 s and absorbs the same push as the
  point-mass scenario. ZMP preview control on fixed footsteps falls sideways
  after about 5 steps *even without the push*: on the real leg dynamics each
  footfall lands a little off, and moving the ZMP inside a 5 cm-wide sole cannot
  absorb the accumulated error without moving the feet. In the point-mass
  scenario, where swing legs are perfect, it only fails under the push.
- Three things were needed beyond the point-mass walker: telling the walker
  where each foot actually landed (`LipmWalker::set_stance_position`),
  switching support on measured contact rather than on the clock, and a
  whole-body controller that keeps contact forces physical (below).
- `rne_wbc::WholeBodyController` solves forces and accelerations as one
  unconstrained least-squares problem and projects the forces onto the friction
  cones only afterwards, so its torques can assume a sole edge pulling on the
  ground; flat-footed walking tipped over with it. `rne::g1::ForceQpWbc`
  distributes the centroidal wrench with a box QP (non-negative normal forces)
  first and then solves the accelerations for those forces. With
  `enforce_torque_limits: true` the RNE controller also returned far too little
  vertical force for a standing G1 (114 N of 315 N), so it is left off there.

## Which controller for which body?

| Body | Model (`models`) | Controllers | Where |
|---|---|---|---|
| Differential drive | `Unicycle` (x, y, yaw, v, w with first-order velocity loops) | pure pursuit, Kanayama, LQR, LTV-MPC (DWA in robo-planning) | `controllers::path_tracking`, `lqr`, `mpc` |
| Car | `Bicycle` (kinematic, rear axle, steering lag) | pure pursuit, Stanley, LQR, LTV-MPC | same |
| Quadruped | single rigid body + massless legs (`scenarios::quadruped` 2D, `legged3d` 3D) | PD wrench + force-distribution QP, convex MPC, Raibert foot placement | `scenarios::quadruped`, `legged3d` |
| Humanoid | 3D linear inverted pendulum with ZMP inside the foot (`scenarios::humanoid`) | ZMP preview control (Kajita) + DCM stabilizer, DCM / capture-point control with step adjustment (`LipmWalker`) | `scenarios::humanoid` |
| Serial arm | `PlanarArmDyn` (N links, recursive Newton-Euler, payload, friction) | PD, PD + gravity compensation, computed torque | `controllers::arm` |
| Drone | `Quadrotor2D` (x-z, two rotors) | cascaded PID, LQR, LTV-MPC with flatness-based feed-forward | `controllers::quadrotor` |
| Inverted pendulum | `CartPole` | energy swing-up + LQR, iLQR + TV-LQR | `controllers::cartpole`, `ilqr` |

Generic building blocks work with any `model::Dynamics`: RK4 integration and
finite-difference linearisation (`model`), DLQR and tracking LQR (`lqr`),
LTV-MPC with a FISTA box-constrained QP (`mpc`), iLQR (`ilqr`), and the
closed-loop simulator with disturbances and seeded noise (`sim`).

## Results

Generated by `body_gallery` (also written to `out/bodies/README.md`).

#### amr

| controller | rms_cte_m | max_cte_m | goal_err_m | time_s | min_clearance_m |
|---|---|---|---|---|---|
| pure pursuit | 0.075 | 0.278 | 0.011 | 39.260 | -0.052 |
| Kanayama | 0.031 | 0.093 | 0.011 | 40.500 | 0.084 |
| LQR | 0.019 | 0.059 | 0.037 | 39.260 | 0.096 |
| LTV-MPC | 0.007 | 0.022 | 0.002 | 39.340 | 0.100 |

#### car

| controller | rms_cte_m | max_cte_m | goal_err_m | time_s | min_clearance_m |
|---|---|---|---|---|---|
| pure pursuit | 0.377 | 1.058 | 1.121 | 53.980 | 4.843 |
| Stanley | 0.402 | 1.353 | 0.868 | 53.980 | 4.307 |
| LQR | 0.615 | 2.228 | 0.015 | 48.100 | 3.704 |
| LTV-MPC | 0.247 | 1.026 | 0.007 | 48.020 | 4.402 |

#### quadruped

| controller | rms_speed_err_mps | rms_height_err_m | rms_pitch_rad | fell |
|---|---|---|---|---|
| PD + force QP | 0.279 | 0.000 | 0.004 | 0.000 |
| convex MPC | 0.155 | 0.003 | 0.012 | 0.000 |

#### humanoid

| controller | fell | fall_time_s | rms_speed_err_mps | zmp_saturated_frac |
|---|---|---|---|---|
| ZMP preview (fixed steps) | 1.000 | 4.635 | 0.062 | 0.556 |
| DCM + step adjustment | 0.000 | NaN | 0.061 | 0.051 |

#### arm

| controller | rms_ee_err_m | max_ee_err_m | final_ee_err_m | min_link_clearance_m |
|---|---|---|---|---|
| PD | 0.500 | 1.131 | 0.188 | -0.150 |
| PD + gravity | 0.110 | 0.229 | 0.095 | 0.064 |
| computed torque | 0.085 | 0.133 | 0.085 | 0.088 |

#### drone

| controller | rms_path_err_m | max_path_err_m | rms_pos_err_m | min_clearance_m |
|---|---|---|---|---|
| cascaded PID | 0.144 | 0.368 | 0.273 | 0.059 |
| LQR | 0.033 | 0.137 | 0.086 | 0.192 |
| LTV-MPC | 0.029 | 0.146 | 0.131 | 0.200 |

#### cartpole

| controller | time_to_upright_s | max_cart_travel_m | final_angle_rad | effort_n2s |
|---|---|---|---|---|
| energy + LQR | 2.270 | 2.275 | 0.009 | 178.802 |
| iLQR + TV-LQR | 2.060 | 1.463 | 0.009 | 256.109 |

Notes on reading the numbers:

- AMR: pure pursuit cuts corners (its minimum clearance is negative: it grazes a shelf); MPC tracks within 2 cm.
- Car: path-based pure pursuit and Stanley slow down on their own and stop short of the goal; time-based LQR and MPC follow the speed profile. LQR without preview cuts hairpins at speed.
- Arm: PD alone sags into the floor (negative clearance) under the unknown payload; computed torque still keeps a small steady error from the payload because it has no integral action.
- Quadruped: the push is simulated as an external force on the body; ground forces are the commanded ones clipped to unilateral contact and the friction cone (no foot slip).
- Humanoid: a fall is declared when the CoM is more than 0.55 m from the stance foot.

## Development

```bash
cargo fmt --all
cargo clippy --all-targets -- -D warnings
cargo test        # every scenario's headline result is asserted in tests/scenarios.rs
cargo test --release -p robo-control-rne   # Go2 / G1 on RNE (first build fetches and compiles RNE, ~4 min)
```

Everything is deterministic: fixed-step RK4 / semi-implicit Euler, no wall clock,
seeded noise.

## License

MIT OR Apache-2.0
