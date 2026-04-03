# Weight & Center of Gravity Parameter Scaling

Complete mapping of how rider weight and center of gravity affect every
tunable parameter in Refloat's control algorithm.

## Physics Foundation

A self-balancing board is an inverted pendulum. Two quantities matter:

- **Weight (mass m)**: Changes torque required per degree of correction and
  rolling resistance. Heavier = more current needed everywhere.
- **COG height (h)**: Changes the pendulum length. Taller COG = slower tipping
  but more torque (leverage) to recover from the same angle.

Key relationships for an inverted pendulum:
- Torque to hold angle θ: `τ = m·g·h·sin(θ) ≈ m·g·h·θ`
- Torque to arrest pitch rate ω: `τ = m·h²·α` (moment of inertia × angular decel)
- Natural frequency: `ω_n = √(g/h)` (independent of mass)

Since motor current is proportional to torque:
- Holding torque per degree ∝ `m·h`
- Rate damping torque per (deg/s) ∝ `m·h²`
- Rolling resistance current ∝ `m`

## Reference Rider

All defaults are assumed tuned for a reference rider:
- **Reference weight**: 80 kg (176 lbs)
- **Reference COG ratio**: 1.0 (unit baseline)

Scaling factors:
```
w  = rider_weight_kg / 80.0       // weight ratio
c  = rider_cog_height / ref_cog   // COG ratio (1.0 = average stance)
wc = w * c                        // combined weight × COG
```

## Complete Parameter Map

### PID Controller

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `kp` | Torque per degree of pitch error | ∝ m·h | `kp *= w * c` |
| `kp2` | Torque per (deg/s) of pitch rate | ∝ m·h² | `kp2 *= w * c * c` |
| `ki` | Steady-state error compensation (slope, wind, COG offset) | ∝ m·h | `ki *= w * c` |
| `ki_limit` | Max integral accumulation, tracks ki | ∝ m·h | `ki_limit *= w * c` |
| `kp_brake` | Ratio multiplier on kp when braking | dimensionless | **no change** |
| `kp2_brake` | Ratio multiplier on kp2 when braking | dimensionless | **no change** |

**Why kp_brake/kp2_brake don't change**: They're multipliers on the already-scaled
kp/kp2. A 1.5× brake multiplier means "50% more aggressive when braking" regardless
of rider weight.

**Why kp2 scales with h²**: kp2 opposes pitch rate. The angular momentum at a given
pitch rate is `L = I·ω = m·h²·ω`. Arresting that momentum requires torque ∝ m·h².
This means taller riders need disproportionately more rate damping — the ratio
kp2/kp grows linearly with COG height.

### PID Tunable: Brake Transition Rate

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `brake_transition_rate` | EMA rate for accel↔brake gain transition | Weak: heavier systems tolerate slower transitions | **no change** |

The system's mechanical time constant is `√(h/g)`, but the difference between
a 1.6m and 1.9m COG height rider is ~9%, which is within noise for a filter
rate. Not worth complicating.

### Balance Filter

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `mahony_kp` | Accelerometer correction gain for pitch | ∝ 1/√h (natural frequency) | `mahony_kp *= 1.0 / sqrtf(c)` |
| `mahony_kp_roll` | Accelerometer correction gain for roll | same | `mahony_kp_roll *= 1.0 / sqrtf(c)` |
| `acc_confidence_decay` | Trust decay for accelerometer | weight-independent (IMU measures board accel, not force) | **no change** |

**Why mahony_kp is COG-dependent but not weight-dependent**: The filter needs to
track the board's oscillation dynamics. Natural frequency ω = √(g/h) depends only
on COG height. A taller COG means slower dynamics, so the filter can correct more
gently (lower kp). Weight doesn't change the oscillation frequency.

**Why acc_confidence_decay doesn't change**: The IMU measures the board's
acceleration (m/s²). For the same speed change, the IMU reads the same value
regardless of rider mass — it just takes more current to achieve that
acceleration with a heavier rider.

### ATR (Adaptive Torque Response)

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `atr_amps_accel_ratio` | Current-to-acceleration model: expected_acc = current / ratio | ∝ m (F=ma, heavier = less accel per amp) | `*= w` |
| `atr_amps_decel_ratio` | Same, for deceleration | ∝ m | `*= w` |
| `atr_strength_up` | Maps accel_diff → setpoint angle | weight-independent if ratio is correct | **no change** |
| `atr_strength_down` | Same, downhill | same | **no change** |
| `atr_threshold_up/down` | Dead band on accel_diff | weight-independent if ratio is correct | **no change** |
| `atr_speed_boost` | Speed-dependent strength multiplier | dimensionless | **no change** |
| `atr_response_boost` | Speed-dependent step size multiplier | dimensionless | **no change** |
| `atr_transition_boost` | Direction change step size multiplier | dimensionless | **no change** |
| `atr_on_speed` / `atr_off_speed` | Ramp rate in degrees/second | geometric, not force-based | **no change** |
| `atr_angle_limit` | Max ATR angle | geometric | **no change** |
| `atr_filter` | Biquad filter frequency | signal processing | **no change** |

**Why ATR strengths don't change**: The `atr_amps_accel_ratio` normalizes the
current-to-acceleration relationship for the rider's mass. Once that's correct,
`accel_diff` represents the same physical meaning (expected minus actual
acceleration in normalized units) regardless of weight. The strengths map
that normalized signal to degrees of tilt — a geometric quantity independent
of mass.

### ATR Tunables (Newly Exposed)

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `torque_offset` | Current to maintain speed (rolling resistance) | ∝ m (F_rr = Crr·m·g) | `= 8.0 * w` |
| `torque_breakpoint` | Motor torque linearity threshold | motor property, not rider | **no change** |
| `torque_breakpoint_scale` | High-current slope reduction | motor property | **no change** |
| `accel_clamp` | Measured acceleration clamp range | ∝ 1/m (a=F/m, heavier = smaller accels) | `= 5.0 / w` |
| `target_smoothing` | ATR target EMA rate | weak relationship | **no change** |
| `winddown_*_rate` | Wheelslip tilt decay rates | weak relationship | **no change** |

### Torque Tilt

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `torquetilt_start_current` | Dead band before tilt activates | ∝ m (heavier rider draws more baseline current) | `*= w` |
| `torquetilt_strength` | Current above threshold → tilt angle | ∝ 1/m (see explanation) | `*= 1.0 / w` |
| `torquetilt_strength_regen` | Same, during regen braking | ∝ 1/m | `*= 1.0 / w` |
| `torquetilt_on_speed/off_speed` | Ramp rate in degrees/second | geometric | **no change** |
| `torquetilt_angle_limit` | Max tilt angle | geometric | **no change** |
| `winddown_rate` | Wheelslip decay rate | weak | **no change** |

**Why torquetilt_strength scales inversely**: On the same hill, a heavy rider draws
proportionally more current. After subtracting start_current (which also scales ∝ m),
the excess current ∝ m. With the same strength, the tilt angle would be ∝ m — the
heavy rider gets more tilt than the light rider on the same hill. Since tilt angle is
geometric feedback (how many degrees the nose lifts), it should be the same for the
same terrain. So strength ∝ 1/m.

### Booster

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `booster_current` | Direct current injection above angle threshold | ∝ m·h (torque to correct angle) | `*= w * c` |
| `brkbooster_current` | Same, when braking | ∝ m·h | `*= w * c` |
| `booster_angle` | Activation threshold in degrees | ∝ 1/(m·h) (heavier rider at same angle = more danger, activate sooner) | `*= 1.0 / (w * c)` |
| `brkbooster_angle` | Same, when braking | ∝ 1/(m·h) | `*= 1.0 / (w * c)` |
| `booster_ramp` | Ramp-up range in degrees | tracks booster_angle | `*= 1.0 / (w * c)` |
| `brkbooster_ramp` | Same, when braking | tracks brkbooster_angle | `*= 1.0 / (w * c)` |

**Why booster_angle is inversely proportional**: A heavier rider at 5° of pitch error
has far more gravitational torque pulling them down than a light rider at the same
angle (τ = m·g·h·sin(5°)). The booster needs to kick in sooner for the heavy rider
because the same angle is more dangerous.

### Brake / Idle

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `brake_current` | Holding current when stopped | ∝ m (resist rider weight tipping board) | `*= w` |
| `startup_click_current` | Tactile feedback click on engage | ∝ m (needs to be felt through weight) | `*= w` |

### Wheelslip Detection (Newly Exposed)

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `wheelslip_accel_start` | Detection threshold | ∝ 1/m (see explanation) | `= 15.0 / w` |
| `wheelslip_accel_end` | Exit threshold | ∝ 1/m (tracks start) | `= 10.0 / w` |
| `wheelslip_scnd_duty` | Secondary duty threshold | motor property | **no change** |
| `wheelslip_timeout` | Hold time after detection | weak | **no change** |

**Why wheelslip thresholds are inversely proportional**: During normal riding,
acceleration = F/m_total. Heavy rider sees lower baseline accelerations. During
actual wheelslip, the wheel spins against its own low inertia — acceleration spikes
regardless of rider weight. So the gap between normal and wheelslip is LARGER for
heavy riders, allowing more sensitive detection (lower threshold). Light riders
need a higher threshold to avoid false positives from their naturally higher
baseline accelerations.

### Current Smoothing (Newly Exposed)

| Parameter | Physics | Scaling | Formula |
|-----------|---------|---------|---------|
| `current_smoothing` | Output current EMA weight on new values | ∝ 1/m (see explanation) | `= clamp(0.2 / w, 0.1, 0.4)` |

Heavier systems have more rotational inertia — they physically can't respond to
current changes as fast. More filtering is safe. Lighter riders need snappier
response (higher smoothing value = less filtering) to prevent nosedives from
delayed corrections.

### Parameters NOT Affected by Weight or COG

These are either geometric (angles/degrees), dimensionless (ratios/multipliers),
signal processing (filter frequencies), or safety thresholds (sensor-based):

- All fault thresholds: `fault_pitch`, `fault_roll`, `fault_adc*`, `fault_delay_*`
- All tiltback angles and speeds: `tiltback_duty_angle`, `tiltback_hv_angle`, etc.
- All nose angling: `tiltback_variable`, `tiltback_constant`, ERPM thresholds
- All turn tilt parameters (yaw-rate based, not force-based)
- Brake tilt strength and lingering (input is angle-based `balance_offset`)
- All ramp speeds (`atr_on_speed`, `torquetilt_on_speed`, `noseangling_speed`, etc.)
- Remote/input tilt parameters
- LED, haptic, BMS, charging parameters
- ATR angle limit, ATR filter frequency
- All winddown rates

## Summary: Scaling Categories

### ∝ m (weight only, linear)
```
torque_offset, atr_amps_accel_ratio, atr_amps_decel_ratio,
torquetilt_start_current, brake_current, startup_click_current
```

### ∝ 1/m (weight only, inverse)
```
torquetilt_strength, torquetilt_strength_regen,
accel_clamp, current_smoothing,
wheelslip_accel_start, wheelslip_accel_end
```

### ∝ m·h (weight × COG)
```
kp, ki, ki_limit, booster_current, brkbooster_current
```

### ∝ m·h² (weight × COG²)
```
kp2
```

### ∝ 1/(m·h) (inverse weight × COG)
```
booster_angle, brkbooster_angle, booster_ramp, brkbooster_ramp
```

### ∝ 1/√h (COG only)
```
mahony_kp, mahony_kp_roll
```

### No change (27 parameters)
```
kp_brake, kp2_brake, brake_transition_rate,
acc_confidence_decay, all ATR strengths/thresholds/boosts/speeds,
torquetilt_angle_limit, torquetilt_on/off_speed,
braketilt_strength, braketilt_lingering, all turn_tilt params,
all tiltback angles/speeds, all fault thresholds,
all nose angling, all winddown rates, target_smoothing,
torque_breakpoint, torque_breakpoint_scale,
wheelslip_scnd_duty, wheelslip_timeout
```

## Implementation: Two-Layer Architecture

### Layer 1: `apply_rider_defaults(weight_kg, cog_ratio, psi)`

Establishes a "default profile" tuned to the rider's setup. The board should
feel the same for a 60kg rider as for a 120kg rider after this is applied.

**Inputs:**
- **Weight (kg)** — rider mass, dominant factor
- **COG ratio** — center of gravity height (1.0 = average, >1 = taller)
- **Tire pressure (PSI)** — affects rolling resistance (F_rr ∝ 1/√psi)

Two strategies are used depending on the parameter type:

#### Direct calculation (new tunables)

These parameters were previously hardcoded constants. Their values are computed
directly from rider inputs — no ratio needed because there's no user-facing
"default" to preserve:

| Parameter | Formula | 80kg/20PSI value |
|-----------|---------|------------------|
| `torque_offset` | `0.1 * weight * √(20/psi)` | 8.0 |
| `accel_clamp` | `400.0 / weight` | 5.0 |
| `current_smoothing` | `clamp(16.0 / weight, 0.1, 0.4)` | 0.2 |
| `wheelslip_accel_start` | `1200.0 / weight` | 15.0 |
| `wheelslip_accel_end` | `800.0 / weight` | 10.0 |

#### Ratio scaling (existing config params)

These are user-tunable values stored in EEPROM. Defaults are assumed tuned
for 80kg at COG ratio 1.0. We multiply by the appropriate ratio so the
rider's tuned values produce the same physical response at their weight:

```
w  = weight / 80.0    // weight ratio
c  = cog              // COG ratio (1.0 = reference)
wc = w * c
```

| Parameter | Ratio |
|-----------|-------|
| `kp`, `ki`, `ki_limit` | `*= wc` |
| `kp2` | `*= wc * c` |
| `mahony_kp` | `*= 1/√c` |
| `mahony_kp_roll` | `*= 1/(√c · psi_factor)` |
| `acc_confidence_decay` | `*= psi_factor` |
| `atr_amps_accel_ratio`, `atr_amps_decel_ratio` | `*= w` |
| `torquetilt_start_current` | `*= w` |
| `torquetilt_strength`, `torquetilt_strength_regen` | `*= 1/w` |
| `turntilt_strength` | `*= 1/psi_factor` |
| `booster_current`, `brkbooster_current` | `*= wc` |
| `booster_angle`, `brkbooster_angle` | `*= 1/wc` |
| `booster_ramp`, `brkbooster_ramp` | `*= 1/wc` |
| `brake_current`, `startup_click_current` | `*= w` |

Where `psi_factor = √(20/psi)`. At 20 PSI (reference) psi_factor = 1.0.

**Tire pressure effects:**
- **Rolling resistance** (`torque_offset`): softer tire = more deformation = more drag
- **Turn tilt** (`turntilt_strength`): softer tire = more self-steering torque = less compensation needed
- **Roll filter** (`mahony_kp_roll`): softer tire damps roll vibrations = cleaner signal
- **Accel confidence** (`acc_confidence_decay`): harder tire transmits more bump noise = trust accel less

### Layer 2: Feel Sliders

Applied on top of the weight-adjusted baseline. Each slider modifies a
subset of parameters to shape ride character (Tightness, Flow, Adaptive,
Carve, Safety, Progression). See RIDE_FEEL_SLIDERS.md for the full design.
