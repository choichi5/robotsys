# Hybridimpedance_3dof — 3 DOF hybrid impedance / force control

3-link planar manipulator extension of `Hybridimpedance_re`.

## Task space

| | 2 DOF (`Hybridimpedance_re`) | 3 DOF (this folder) |
|---|---|---|
| joints | `q11, q12` | `q11, q12, q13` |
| task vector `X` | `[x ; y]` | `[x ; y ; phi]`, `phi = q11+q12+q13` |
| Jacobian | 2×2 | 3×3 (still square → `J \ (...)` unchanged) |

Keeping the Jacobian square means the resolved-acceleration law
`dd_q = J \ (dd_x - dJ*dQ)` and the whole hybrid control structure carry over
without pseudo-inverses or null-space terms.

## Control split (in contact with the wall at `x = -0.5`)

| direction | law | gains |
|---|---|---|
| `x` | force control, regulates contact force to `F_d = 10 N` | `K_mf, K_df, A_f, B_f` |
| `y` | position control, slides along the wall | `K_mp, K_dp, K_pp` |
| `phi` | position control, keeps the tool normal to the wall | `K_mo, K_do, K_po` |

Out of contact all three directions use the same computed-torque law as the
2 DOF model. The contact model is a point contact: stiff normal spring/damper
in `x`, viscous friction in `y`, zero contact moment.

## Scenario

Start `q = (30°, 60°, 110°)` → `X0 = [-0.074 ; 1.158 ; 200°]` — clear of the
wall and 20° off normal. The cubic trajectory drives the tool to
`[-1.0 ; 0.5 ; 180°]` over 2 s, so the arm approaches the wall, rotates the
tool to normal, makes contact around `t ≈ 0.93 s`, then slides down it while
holding 10 N.

## Files

| file | change vs. 2 DOF |
|---|---|
| `init.m` | adds `Link13`, `Mass13`, `I13`, `Link13g`; `Q`, `dQ` are 3-vectors |
| `direct_kinematics.m` | 3-link FK, 3×3 Jacobian, `IN` 6×1, `OUT` 6×1 |
| `desired.m` | cubic in `x, y, phi`; `OUT` 1×9 |
| `circle.m` | same circular path, `OUT` 1×9 with `phi_d = pi` |
| `Dynamics.m` | 3×3 `H`, Christoffel-based `S`, 3×1 `G`, 3×3 `J`/`dJ`; `IN` 22×1, `OUT` 19×1 |
| `controller.m` | same matrices; pass-through `IN`/`OUT` 22×1 |
| `test_avi.m` | draws three links |
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
  `T_final`. Evaluating the cubic at `t = T_final` (what the 2 DOF file does)
  leaves `dds = -1.5`, i.e. an acceleration command that keeps acting after the
  motion is over.
- `circle.m` acceleration now includes the missing `-r*cos(omega)*vomega^2`
  centripetal term (and the `r` factor) in `ax`/`ay`.
- The free-space gains are carried over unchanged: `K_d = 100` multiplies the
  **velocity** error and `K_p = 1` the **position** error, i.e. the header
  comments in the original file are swapped relative to actual use.
