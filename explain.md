# 16-axis-program-explain
These values are gait-tuning constants rather than values derived from a single mechanical equation. They establish the shape, speed and safety margin of the implemented **2D lateral undulation plus lift bias** gait. The source code demonstrates their effects, but it does not document how the exact numbers were selected. They are therefore best treated as prototype tuning choices, possibly refined experimentally.

| Constant | Purpose and effect |
|---|---|
| `WAVE_AMPLITUDE_DEG = 25.0f` | Sets the maximum nominal yaw-wave deflection to ±25°. At full drive, the wave is still scaled by the spatial envelope `(0.75 + 0.25 s_i)` and the amplitude-trim factor `0.8 + 0.2r`. This produces a visible lateral wave without using the full ±60° deflection allowed by the 30–150° servo range. |
| `WAVE_FREQUENCY_HZ = 1.15f` | Defines the nominal lateral-wave frequency at full drive. One complete phase cycle takes `1 / 1.15 ≈ 0.87 s`. A lower value would make the robot slow to respond; a much higher value would demand faster movement from the servos, cables and joints. |
| `WAVE_PHASE_LAG_RAD = 0.72f` | Introduces the delay between adjacent yaw joints. It corresponds to `0.72 × 180 / π ≈ 41.25°` per joint and about `288.8°` between the first and eighth joints. This phase difference makes the lateral bend move along the body instead of all joints bending in unison. |
| `STEER_BIAS_DEG = 20.0f` | Adds a left-right yaw offset for steering. The code applies full steering near the head and reduces it toward the tail through `(1 - 0.35s_i)`, where `s_i` is the normalised joint position. This lets the head initiate a turn while the rest of the body retains the gait. |
| `PITCH_BIAS_DEG = 16.0f` | Sets the maximum vertical lift offset. The pitch envelope `(1 - 0.45s_i)` gives the head the largest lift and the tail the smallest. Unlike the yaw command, pitch does not contain a sinusoidal phase term, so this is a lift bias rather than a second travelling wave. |
| `MAX_ANGLE_RATE_DPS = 110.0f` | Limits how quickly every servo command may change. At the 20 ms control period, the maximum command change per update is `110 × 0.020 = 2.2°`. A full 60° change from neutral therefore takes approximately `60 / 110 = 0.545 s`. This reduces mechanical shock and simultaneous current peaks from sixteen servos. |
| `CONTROL_PERIOD_S = 0.020f` | Sets a nominal 50 Hz command-update rate. This matches the PCA9685’s nominal 50 Hz servo-PWM period, so the program generates new targets at approximately one servo period per update. The ultrasonic and OLED tasks run outside this gait gate. |
| `TWO_PI_RAD = 6.283185307179586f` | Represents one complete phase cycle in radians. It converts the 1.15 Hz frequency into the phase increment and wraps the phase after each cycle: `phase += 2πf·drive·Δt`. |

The yaw amplitude and steering values also provide a useful safety margin. The strongest yaw contribution per joint is approximately:

\[
25(1.0)(1.0)+20(1.0)=45^\circ
\]

only if both envelopes were simultaneously equal to one. In the actual code, the wave envelope is strongest at the tail while the steering envelope is strongest at the head. The largest per-joint combination is approximately `38.75°`, so the normal gait remains well inside the ±60° deflection limit and therefore inside the absolute 30–150° command range.

In practical terms, these constants were likely chosen to give a visible but controllable body wave, enough steering authority, a modest head lift and a safe command rate for the MG90S servos and cable transmission. They should be changed only after measuring wave shape, turning response, current and joint tracking on the physical robot.
