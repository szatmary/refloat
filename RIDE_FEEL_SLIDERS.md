# Ride Feel Sliders

A proposal for 6 high-level sliders that control the feel of the board, plus a
weight calibration input. Each slider maps to multiple underlying parameters.
A slider is not 1:1 with any single setting.

## Calibration Input (Set Once)

### Rider Weight (kg/lbs)

Not a slider — a setup input that rarely changes. Tunes the motor model to
match the rider's mass.

| Parameter | Effect |
|-----------|--------|
| ATR torque offset (currently hardcoded 8A) | Scales linearly with weight |
| `atr_amps_accel_ratio` / `atr_amps_decel_ratio` | Higher for heavier riders |
| ATR measured accel clamp (currently hardcoded ±5) | Wider for light, narrower for heavy |
| ATR torque linearity breakpoint (currently hardcoded 25A) | Higher for heavier riders |
| `torquetilt_start_current` | Higher for heavier riders (more baseline current draw) |
| `torquetilt_strength` / `torquetilt_strength_regen` | Lower for heavier riders |
| `booster_current` / `brkbooster_current` | Scale with weight |
| `startup_click_current` | Scale with weight |
| `brake_current` | Scale with weight |

---

## Slider 1: Tightness

**How locked-in the balance feels.**

Loose / Surfy (0) ←→ Locked-In / Planted (10)

Controls the core balance loop — how hard the board fights to stay level. A
loose board flows under you; a tight board feels bolted to your feet.

| Parameter | Loose (0) → Tight (10) |
|-----------|------------------------|
| `kp` | lower → higher |
| `kp2` (Rate-P) | lower → higher |
| `kp_brake` | lower → higher |
| `kp2_brake` | lower → higher |
| `mahony_kp` | lower → higher |
| Output current smoothing (currently hardcoded 0.8/0.2) | more smoothing → less smoothing |
| Soft start ramp (currently hardcoded 100) | slower → faster |
| `booster_angle` | wider → narrower |

**Rider question**: "How does it feel standing still and at steady speed?"

---

## Slider 2: Flow

**How gracefully the board transitions between states.**

Snappy / Direct (0) ←→ Buttery / Graceful (10)

Controls all ramp rates and transition filters. Nothing about magnitude — only
about how smoothly the board moves between acceleration, braking, flat, hill,
and turn states.

| Parameter | Snappy (0) → Flowing (10) |
|-----------|---------------------------|
| PID brake transition rate (currently hardcoded 0.01/0.99) | faster (0.05/0.95) → slower (0.005/0.995) |
| `atr_on_speed` / `atr_off_speed` | faster → slower |
| `torquetilt_on_speed` / `torquetilt_off_speed` | faster → slower |
| `noseangling_speed` | faster → slower |
| ATR `transition_boost` | higher → lower |
| ATR target smoothing (currently hardcoded 0.95/0.05) | less smoothing → more smoothing |
| Winddown rates (currently hardcoded 0.995/0.99) | faster decay → slower decay |

**Rider question**: "How does it feel when it changes what it's doing?"

---

## Slider 3: Adaptive

**How actively the board compensates for terrain and resistance.**

Passive / Rider-Does-It (0) ←→ Active / Board-Does-It (10)

Controls the magnitude of all tilt adjustments that respond to external forces
— hills, headwinds, rough ground, surface changes. At low settings, the rider
manages terrain with their body. At high settings, the board does it for them.

| Parameter | Passive (0) → Active (10) |
|-----------|---------------------------|
| `atr_strength_up` / `atr_strength_down` | lower → higher |
| `atr_speed_boost` | lower → higher |
| ATR `response_boost` | lower → higher |
| `torquetilt_strength` / `torquetilt_strength_regen` | lower → higher |
| `braketilt_strength` | lower → higher |
| ATR measured accel clamp (currently hardcoded ±5) | tighter → wider |

**Rider question**: "How does it handle hills and rough ground?"

---

## Slider 4: Carve

**How the board behaves through turns.**

Neutral / Flat (0) ←→ Surfy / Banks-Into-Turns (10)

Controls the turn tilt system exclusively. This is the most isolated slider —
turn tilt parameters don't overlap with the other feel dimensions.

| Parameter | Neutral (0) → Surfy (10) |
|-----------|--------------------------|
| `turntilt_strength` | lower → higher |
| `turntilt_angle_limit` | lower → higher |
| `turntilt_start_angle` | higher → lower |
| `turntilt_start_erpm` | higher → lower |
| `turntilt_erpm_boost` | lower → higher |
| `turntilt_speed` | lower → higher |
| `turntilt_yaw_aggregate` | higher → lower |

**Rider question**: "How does it handle turns?"

---

## Slider 5: Safety

**How aggressively the board protects the rider.**

Relaxed / Trusts-Rider (0) ←→ Protective / Assertive (10)

Controls all safety feedback — pushback, haptics, speed limits, fault
sensitivity, and wheelslip detection. A relaxed board gives subtle hints. A
protective board makes limits unmistakable.

| Parameter | Relaxed (0) → Protective (10) |
|-----------|-------------------------------|
| `tiltback_duty_angle` / `tiltback_duty_speed` | lower → higher |
| `tiltback_hv_angle` / `tiltback_lv_angle` | lower → higher |
| `tiltback_hv_speed` / `tiltback_lv_speed` | lower → higher |
| `tiltback_speed` (speed limit) | higher / off → lower |
| Haptic `duty.strength` / `vibrate.strength` | lower → higher |
| Haptic `error.strength` | lower → higher |
| Wheelslip accel threshold (currently hardcoded 15) | higher → lower (more sensitive) |
| Wheelslip exit timeout (currently hardcoded 0.2s) | shorter → longer |
| `fault_delay_switch_half` / `fault_delay_switch_full` | longer → shorter |

**Rider question**: "How hard does it try to keep me safe?"

---

## Slider 6: Progression

**How the board's character changes with speed.**

Flat / Same-At-All-Speeds (0) ←→ Progressive / Stiffens-With-Speed (10)

Controls speed-dependent scaling across the system. Tightness sets the
baseline; Progression sets the slope. Two riders with identical Tightness at
5mph can have completely different boards at 25mph.

| Parameter | Flat (0) → Progressive (10) |
|-----------|----------------------------|
| `atr_speed_boost` | 0 → configured value |
| ATR `response_boost` | 1.0 → configured value |
| ATR speed boost onset (currently hardcoded 3000 ERPM) | higher → lower |
| Booster speed stiffness scaling | flat → ramping |
| `tiltback_variable` | 0 → configured value |
| `tiltback_variable_max` | low → high |
| `tiltback_constant` | 0 → configured value |
| `kp_brake` at speed | less scaling → more scaling |
| Nose angling ERPM thresholds | higher → lower |

**Rider question**: "Does the board feel the same at 25mph as at 5mph?"

---

## Design Principles

### Orthogonality

Each slider answers a different question. No two sliders compete or conflict:

| Slider | Domain |
|--------|--------|
| Tightness | Balance loop gain (baseline) |
| Flow | Transition ramp rates |
| Adaptive | Tilt adjustment magnitudes |
| Carve | Turn tilt system |
| Safety | Protection thresholds and feedback |
| Progression | Speed-dependent scaling |

### Independence

Moving one slider should not require adjusting another. Specifically:

- Tightness + Flow: A rider can be tight but flowing, or loose but snappy
- Tightness + Progression: Tightness sets 5mph feel, Progression sets how much it changes at 25mph
- Adaptive + Carve: Terrain compensation and turn feel are independent systems
- Safety + everything: Protection level is a personal risk tolerance, independent of ride style

### Weight Separation

Rider weight is a calibration input, not a feel preference. It adjusts the
physics model so that the 5 feel sliders produce the same *perceptual* result
regardless of rider mass. A 70kg rider at Tightness=5 should feel similar to a
110kg rider at Tightness=5.

### Slider-to-Parameter Mapping

Sliders should use non-linear mapping (polynomial or piecewise) to parameters.
Many parameters have non-linear perceptual impact. Suggested form:

```
param = base + (max - base) * powf(slider_normalized, exponent)
```

Where `exponent` is tuned per-parameter so that perceptual change is roughly
linear across the slider range.

### Base Profiles

Rather than computing from scratch, define 2-3 anchor configs (e.g., Beginner,
Street, Trail) as complete parameter sets. Sliders modify from the nearest
anchor. This avoids pathological parameter combinations and gives sensible
starting points.
