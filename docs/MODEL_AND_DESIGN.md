# Model, frames, sensors and tuning rationale

## Robot model

The simulated robot is a TurtleBot3-like differential-drive platform with nominal wheel radius `r=0.033 m` and axle separation `b=0.160 m` (confirm for the exact robot model and wheel mounting). Control input is wheel angular rates `φ̇_L, φ̇_R`, or equivalently `v, ω`:

- `v = r/2 * (φ̇_R + φ̇_L)`
- `ω = r/b * (φ̇_R − φ̇_L)`
- `ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω`.

These are planar nonholonomic kinematics. They are *not* a complete electromechanical motor dynamics model; wheel contact/rigid-body dynamics are provided by MuJoCo when that backend is used. The locally linearized trajectory tracking model uses tracking error `e = [e_x, e_y, e_θ]^T` in the body frame, `δu = [v−v_ref, ω−ω_ref]^T`:

```
ė = A e + B δu
A = [[0, ω_ref, 0], [-ω_ref, 0, v_ref], [0, 0, 0]]
B = [[-1, 0], [0, 0], [0, -1]]
A_d ≈ I + dt A; B_d = dt B
```

The feedback is `δu = −K e` with `K = (R + B_d^T P B_d)^−1 B_d^T P A_d`, where `P` solves the discrete algebraic Riccati equation. With `v_ref > 0`, the linearized 3-state pair `(A,B)` has controllability rank 3; with `v_ref = 0`, it falls to 2. This rank is *local*, not a claim of global controllability under nonholonomic constraints. Reproduce with `python scripts/analyse_model.py`.

## Observability and EKF

Let `L = [L_x,L_y]` be the known **world-frame** landmark coordinate for the ArUco marker. Measurement is camera-derived base-to-marker planar range and bearing:

```
Δx = L_x − x,  Δy = L_y − y
h(x) = [sqrt(Δx² + Δy²), wrap(atan2(Δy,Δx) − θ)]
```

For state `x=[x,y,θ]^T`, the Jacobian `H=∂h/∂x` is:

```
[ −Δx/ρ,    −Δy/ρ,    0 ]
[  Δy/ρ²,  −Δx/ρ²,   −1 ]
```

This measurement has rank 2 at any single instant, so pose is not fully observable from **one static landmark observation alone**. The locally stacked observability matrix formed with robot translation at nonzero reference speed can have rank 3. Wheel odometry supplies motion prediction; persistent range/bearing measurements constrain drift. When the landmark is absent, EKF uses odometry prediction and covariance grows; global pose observability weakens. When stationary with only one marker, one pose degree of freedom remains unobservable. Do not overstate the result as globally observable.

Process: measured encoder increments `dφ_L, dφ_R` yield `ds=r(dφ_R+dφ_L)/2` and `dθ=r(dφ_R−dφ_L)/b`. Midpoint integration propagates the pose; Jacobians propagate covariance from encoder noise. Camera: `z=[ρ,β]` corrects via EKF, `S=H P H^T + R`; normalized innovation squared (NIS) is gated at 13.82 (approx. 99.9% for 2D Gaussian innovations). The covariance update uses Joseph form to reduce numerical error. Angle wrapping is applied to innovations and state.

## Camera frame calibration

- `world`: planar coordinate system of the room.
- `base_link`: robot x forward, y left, z up.
- `camera`: optical convention x right, y down, z forward (OpenCV).
- `marker`: ArUco target, marker ID 0, nominal square size 0.22 m.

The MuJoCo scene places the RGB camera 0.105 m forward of `base_link` by default (`camera_offset_x_m`, configurable), pointing in the base's x-forward direction. For pinhole intrinsics derived from vertical field of view and image resolution, `fy = H/(2 tan(fovy/2))`, `fx=fy`, `(cx,cy)=(W/2,H/2)`, with zero distortion **in simulation**. The image detector solves marker pose with `solvePnP(SOLVEPNP_IPPE_SQUARE)`; optical depth becomes robot-frame forward; `−x_optical` becomes robot-frame lateral; the configured camera extrinsic is added in the base frame. In hardware, use `/camera/camera_info` for `K,D`, confirm the physical camera-to-base transform, and replace the example extrinsic if needed. Test signs visually and against measured range/bearing. Lens calibration via checkerboard was **not conducted** in this development environment.

## LiDAR sensing and mapping

MuJoCo 2D scan consists of multiple horizontal rangefinder rays attached around the robot perimeter; the `kinematic` test double raycasts against ideal rectangles. Scan uses angles relative to `base_link`, fixed maximum range and Gaussian noise. The navigator increments a **sensor-derived** occupancy grid via inverse log-odds updates. Obstacle cells are inflated by a robot-body safety margin, and an 8-connected A* search plans grid-cell waypoints. A separate front-sector LiDAR guard brakes/turns near obstacles. The obstacle geometry is **not** passed into the planner, even though the simulation backend uses geometry for physics/ray sensors.

Limitations: the hardware adapter assumes LiDAR polar measurements already align with the base heading (no measured transform), occupancy integration is approximate and not a full SLAM system, and the MuJoCo LiDAR model differs from real LDS scanning time and reflectivity.

## Tuning and sensitivity plan

| Quantity | Initial baseline | Engineering interpretation / study |
|---|---:|---|
| EKF encoder noise | 0.003 rad + motion-dependent term | Increases predicted covariance during odometry motion; compare static/moving encoder residuals |
| EKF range SD | 0.045 m | Represents camera/marker detection uncertainty; derive experimentally using fixed-distance measurements |
| EKF bearing SD | 0.028 rad | Represents projected-angle uncertainty; calibrate from marker detections at known poses |
| LQR Q | diag(4, 18, 10) | Penalises lateral displacement and heading more than longitudinal lag |
| LQR R | diag(1.5, 1.1) | Penalises command corrections; sweep ±50% for tracking/control-effort trade-off |
| Velocity bounds | 0.22 m/s, 1.6 rad/s | Conservative software limits, physical hardware further capped |
| Map cell | 0.12 m | Trade-off between resolution and speed; vary with inflation radius |
| Obstacle inflation | 0.16 m | Approximate collision envelope, check clearance in contact physics |

These values are **design assumptions** pending quantitative calibration and tuning in the MuJoCo simulation. For a defensible D/HD report: 5–10 random seeds per scenario, RMSE, success/collision rate, final error, control effort, NIS and percent of residual inside 2σ. Compare 0.5×, 1×, 2× camera-noise covariance; discuss consistency and responsiveness. Compare no-EKF odometry vs fused EKF if adding an independent extension.
