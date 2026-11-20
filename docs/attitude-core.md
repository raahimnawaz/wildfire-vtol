# Attitude core: design notes

Details behind the [README](../README.md): the conventions the code pins down, how the gains were chosen, and how the loop stays fast on a chip without an FPU.

## Conventions

Most quaternion bugs are two individually-correct pieces of code disagreeing
about convention. This project pins them:

| Thing | Choice |
|---|---|
| Storage order | `q = [w, x, y, z]` — scalar first |
| Algebra | Hamilton, not JPL |
| Meaning of `q` | rotates a vector **from body frame into world frame** |
| `ω`, `τ` | body frame; rad/s and N·m |
| Wire format | little-endian `float32` |
| Angles on the wire | radians — degrees only for display |

Euler angles are computed for the OLED and **never fed back into control**.

## Gains

Treating each axis as `J·θ̈ = τ` and using the small-angle result `q_err.vec ≈ θ/2`,
the effective proportional gain is `Kp/2`:

```
ωn = sqrt(Kp / (2·J))          ζ = Kd / (2·sqrt(Kp·J/2))
```

With `J = [1, 2, 3]`, the shipped gains `Kp = [18, 36, 54]` and
`Kd = [4.2, 8.4, 12.6]` give **ωn = 3.0 rad/s and ζ = 0.7 on all three axes**.

The front-panel knob scales `Kp` and `Kd` together by `m ∈ [0.1, 3.0]`. Because
`ζ` depends on `Kd/sqrt(Kp)`, scaling both by `m` scales damping by `√m` — so the
knob sweeps ζ from 0.22 (ringing) to 1.21 (sluggish), passing through 0.7 at
m = 1. The knob is a damping control wearing a gain control's clothing.

## Timing

Nothing on the control path blocks. The I²C bus runs at 400 kHz, the OLED
framebuffer streams two 31-byte chunks per loop instead of one 25 ms
`display()`, and telemetry is printed as fixed point rather than soft-float
`Serial.print`. On the AVR's soft float, `invSqrt` (the Quake III bit trick, one
Newton step) normalises the accelerometer in 730 cycles against 979 for
`1.0f/sqrtf`. The quaternion uses `1.0f/sqrtf`, because a second Newton step
costs more than the call it replaces.
