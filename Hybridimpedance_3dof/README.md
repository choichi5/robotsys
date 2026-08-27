# Hybridimpedance_3dof — 3 DOF hybrid impedance / force control

3-link planar manipulator extension of `Hybridimpedance_re`.

> 한국어로 된 자세한 설명은 **[GUIDE.md](GUIDE.md)** 를 보세요. 이 README 는
> 구조·수치 요약이고, GUIDE 는 왜 이렇게 만들었는지까지 풀어 쓴 문서입니다.

## Task space

| | 2 DOF (`Hybridimpedance_re`) | 3 DOF (this folder) |
|---|---|---|
| joints | `q11, q12` | `q11, q12, q13` |
| task vector `X` | `[x ; y]` | `[x ; y ; phi]`, `phi = q11+q12+q13` |
| Jacobian | 2×2 | 3×3 (still square → `J \ (...)` unchanged) |

Keeping the Jacobian square means the resolved-acceleration law
`dd_q = J \ (dd_x - dJ*dQ)` and the whole hybrid control structure carry over
without pseudo-inverses or null-space terms.

## Environment

Both set in `init.m`:

| | value | notes |
|---|---|---|
| `x_wall` | `-1.5` | moved out from the 2 DOF value of `-0.5`; a 3-link arm reaches 3 m and was starting almost on top of the wall |
| `y_ground` | `0.0` | no part of the arm may go below it |

### Ground constraint

The 2 DOF model had no ground at all, so links were free to swing below
`y = 0`. `Dynamics.m` now applies a penalty contact to the **whole arm**, not
just the tool: each link contributes two contact points (midpoint and end
point, six total). For any point below `y_ground`,

```
f_gy = max( -(k_g*(p_y - y_ground) + d_g*v_y), 0 )   % unilateral, pushes only
f_gx = -mu_g*v_x                                     % friction
Tau_g += Jp' * [f_gx ; f_gy]
```

with `k_g = 1e6`, `d_g = 1e2`, `mu_g = 20`. `Tau_g` is added to the joint
acceleration as an **uncompensated disturbance** — the controller does not know
about it, which is physically correct for an unexpected collision.

Verified by commanding the tool to `y = -0.8`: the arm stops at `-0.77 mm`
(penalty penetration) with the elbow and tool resting on the floor, instead of
passing through.

## Control split (in contact with the wall at `x = x_wall`)

| direction | law | gains |
|---|---|---|
| `x` | force control, regulates contact force to `F_d = 10 N` | `K_mf, K_df, A_f, B_f` |
| `y` | position control, slides along the wall | `K_mp, K_dp, K_pp` |
| `phi` | position control, keeps the tool normal to the wall | `K_mo, K_do, K_po` |

Out of contact all three directions use the same computed-torque law as the
2 DOF model. The contact model is a point contact: stiff normal spring/damper
in `x`, viscous friction in `y`, zero contact moment.

## Scenario

Start `q = (90°, 0°, 0°)` — arm fully extended along `+y`, so
`X0 = [0 ; 3 ; 90°]`. A quintic trajectory drives the tool to
`[-2.0 ; 0.5 ; 180°]` over 2 s. The commanded `x` is 0.5 m *past* the wall, so
the arm reaches the wall at `t ≈ 1.30 s` (at `y ≈ 1.13`), the force loop takes
over in `x`, and the tool then slides about **0.63 m** down the wall face to
`y = 0.5` while holding 10 N.

### Singularity at the start pose

A fully extended planar arm is **singular**: `det J = 0`, because the arm
cannot accelerate along its own axis. Straight up along `+y`, the second row of
`J` is all zeros. Two changes make the extended start pose usable:

1. **`desired.m` uses a quintic, not a cubic.** A quintic has zero velocity
   *and* zero acceleration at `t = 0`, so no task-space acceleration is
   demanded while `J` is still rank deficient. A cubic has `dds(0) = 6/T² ≠ 0`
   and would demand infinite joint acceleration at `t = 0`.
2. **`Dynamics.m` inverts `J` by damped least squares.** When the smallest
   singular value drops below `eps_dls = 0.10`, the damping
   `lam2 = lam_0²(1 - (sigma_min/eps_dls)²)` is blended in:

   ```
   dd_q = (J'J + lam2*I) \ (J'*(dd_x - dJ*dQ))
   ```

   Away from singularities `lam2 = 0` and this is exactly the original
   `J\(dd_x - dJ*dQ)`. `controller.m` likewise uses `pinv` instead of `inv`,
   and `FF` uses the same damped inverse of `J'`.

## Files

| file | change vs. 2 DOF |
|---|---|
| `init.m` | adds `Link13`, `Mass13`, `I13`, `Link13g`, `x_wall`, `y_ground`; `Q`, `dQ` are 3-vectors; starts fully extended |
| `direct_kinematics.m` | 3-link FK, 3×3 Jacobian, `IN` 6×1, `OUT` 6×1 |
| `desired.m` | quintic in `x, y, phi`; `OUT` 1×9 |
| `circle.m` | same circular path, `OUT` 1×9 with `phi_d = pi` |
| `Dynamics.m` | 3×3 `H`, Christoffel-based `S`, 3×1 `G`, 3×3 `J`/`dJ`; ground contact; DLS inverse; `IN` 22×1, `OUT` 19×1 |
| `controller.m` | same matrices, `pinv` instead of `inv`; pass-through `IN`/`OUT` 22×1 |
| `test_avi.m` | full robot-arm rendering: per-link colours, joint caps, base, gripper, wall/ground hatching, tool trail, live force readout |
| `run.m` | new `data` column layout, extra orientation subplot |
| `Impedance_model.mdl` | all Mux/Demux widths and MATLAB Fcn output dimensions rescaled |

### Signal widths in `Impedance_model.mdl`

| block | 2 DOF | 3 DOF |
|---|---|---|
| `Mux` (→ controller) | `[6 4 4 1]` = 15 | `[9 6 6 1]` = 22 |
| `controller` OutputDimensions | 15 | 22 |
| `Robot Dynamics` OutputDimensions | 13 | 19 |
| `Demux` | `[8 1 2 2]` | `[12 1 3 3]` |
| `Demux1` | `[4 2 2]` | `[6 3 3]` |
| `Direct Kinematics` OutputDimensions | 4 | 6 |
| `Integrator/Mux` | `[2 2]` | `[3 3]` |
| `Mux1` | `[8 2]` | `[12 3]` |
| `Save Data/Demux` | `[4 2 2 2]` | `[6 3 3 3]` |
| `Save Data/Mux` | `[1 4 2 2]` = 9 | `[1 6 3 3]` = 13 |

### `data` columns

```
 1     t
 2: 4  x, y, phi          actual task-space pose
 5: 7  xd, yd, phid       desired task-space pose
 8:10  q11, q12, q13      [deg]
11:13  Fx, Fy, Mz         contact wrench
```

## Running

```matlab
run
```

## Notes on differences from the 2 DOF files

- `desired.m` forces velocity and acceleration feed-forward to zero after
  `T_final`. Evaluating the polynomial at `t = T_final` (what the 2 DOF file
  does) leaves `dds = -1.5`, i.e. an acceleration command that keeps acting
  after the motion is over.
- `circle.m` acceleration now includes the missing `-r*cos(omega)*vomega^2`
  centripetal term (and the `r` factor) in `ax`/`ay`.
- The free-space gains are carried over unchanged: `K_d = 100` multiplies the
  **velocity** error and `K_p = 1` the **position** error, i.e. the header
  comments in the original file are swapped relative to actual use.
- The in-contact `y` loop is also carried over unchanged, and it is very
  underdamped: `K_pp = 100` with `K_dp = 1` gives `wn = 10 rad/s` and
  `zeta = 0.05`. That is the visible ringing in `F_y` after `t = 2 s`. Raising
  `K_dp` to about 10 would critically damp it, but that is a retune, not a DOF
  change, so it is left alone.
