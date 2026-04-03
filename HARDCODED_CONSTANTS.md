# Hardcoded Constants in Refloat Control Algorithm

An inventory of hardcoded numerical constants in the control loop that are not
exposed as configuration parameters, organized by impact and tunability.

## Tier 1: Directly Weight/Rider-Sensitive — Strong Case for Tuning

### 1. ATR Torque Offset — 8A

**Location**: `src/atr.c:55`
```c
float torque_offset = 8;  // hard-code to 8A for now (shouldn't really be changed much anyways)
```

The current required to maintain speed on flat ground (overcoming rolling
resistance + drag). Subtracted from motor current before computing expected
acceleration. The comment acknowledges the hardcoding.

**Why it matters**: Rolling resistance is proportional to rider weight. An 80kg
rider might need 5A to maintain speed; a 120kg rider might need 12A. With this
fixed at 8A, the ATR algorithm systematically misjudges expected acceleration
for lighter and heavier riders. For a heavy rider, ATR under-predicts expected
acceleration (thinks the rider is going uphill when they're not), causing
phantom nose-lift.

**Recommendation**: Configurable parameter in ATR settings. Default 8A.

---

### 2. ATR Torque-to-Acceleration Linearity Breakpoint — 25A / 1.3x

**Location**: `src/atr.c:68-74`
```c
if (abs_torque < 25) {
    expected_acc = (motor->filt_current - motor->erpm_sign * torque_offset) / accel_factor;
} else {
    expected_acc = (torque_sign * 25 - motor->erpm_sign * torque_offset) / accel_factor;
    expected_acc += torque_sign * (abs_torque - 25) / accel_factor2;
}
```
Where `accel_factor2 = accel_factor * 1.3`.

Models motor saturation: below 25A the torque-acceleration relationship is
linear, above it the slope decreases by 30%. Both the breakpoint (25A) and the
slope reduction factor (1.3x) are hardcoded.

**Why it matters**: The motor saturation point depends on the motor, battery,
and rider weight. A rider on a high-torque motor with a large battery pack may
not saturate until 35-40A. The 1.3 factor is a rough approximation of the
non-linearity.

**Recommendation**: Breakpoint (25A) should be configurable or auto-detected.
The 1.3 slope factor is less critical but would benefit from tuning.

---

### 3. ATR Measured Acceleration Clamp — [-5, 5]

**Location**: `src/atr.c:62-63`
```c
float measured_acc = fmaxf(motor->acceleration, -5);
measured_acc = fminf(measured_acc, 5);
```

Clamps measured acceleration to prevent outliers from distorting ATR.

**Why it matters**: A lightweight rider on a powerful motor can legitimately hit
accelerations above 5 (ERPM-delta units). Clamping clips real data. A heavy
rider rarely exceeds 3. This should scale with the system's mass.

**Recommendation**: Scale with a rider weight parameter or make configurable.

---

### 4. Output Current Smoothing — 0.8/0.2

**Location**: `src/main.c:946`
```c
d->balance_current = d->balance_current * 0.8 + new_current * 0.2;
```

First-order lowpass filter on the final motor current output. At 800Hz, this
gives an effective time constant of ~4 cycles (~5ms).

**Why it matters**: Heavier riders have more inertia, so the motor can afford to
change current more gradually (higher smoothing). Lighter riders need faster
response to prevent nosedives. This is a core ride-feel parameter that is
invisible to users.

**Recommendation**: Configurable as "current smoothing" with a 0.0-1.0 range.

---

## Tier 2: Significant Ride Feel Impact — Good Candidates for Tuning

### 5. PID Brake Scale Transition Rate — 0.01/0.99

**Location**: `src/pid.c:51-66`
```c
pid->kp_brake_scale = 0.01 * config->kp_brake + 0.99 * pid->kp_brake_scale;
```

Controls how fast the PID transitions between acceleration and braking gains.
At 800Hz this takes ~400 cycles (~500ms) to reach 98% of target.

**Why it matters**: Controls how abruptly the board switches between
acceleration and braking feel. A rider wanting crisp braking response wants
faster transition; a rider wanting smooth carving wants slower. Currently fixed.

**Recommendation**: Configurable as "brake transition speed".

---

### 6. PID Brake Scale ERPM Threshold — 500

**Location**: `src/pid.c:49`
```c
if (md->abs_erpm < 500) {
    // all scaling should roll back to 1.0 when near a stop for smooth transitions
```

Below 500 ERPM, brake scaling resets to 1.0 for smooth stop transitions. This
threshold determines when the board starts "feeling different" at low speed.

---

### 7. Wheelslip Detection Thresholds

**Location**: `src/main.c:546-549`
```c
fabsf(d->motor.acceleration) > 15 &&
sign(d->motor.acceleration) == d->motor.erpm_sign && d->motor.duty_cycle > 0.3 &&
d->motor.abs_erpm > 2000
```

Three hardcoded thresholds for detecting wheelslip:
- Acceleration > 15 (ERPM-delta units)
- Duty cycle > 0.3
- Absolute ERPM > 2000

**Why it matters**: A heavier rider on loose terrain (sand, gravel) will see
different acceleration signatures than a lightweight rider on pavement. The
acceleration threshold in particular is weight-sensitive — heavier riders have
more inertia, making genuine wheelslip look less dramatic in ERPM terms.

**Recommendation**: At minimum the acceleration threshold (15) should be
tunable.

---

### 8. Wheelslip Exit Conditions

**Location**: `src/main.c:557-568`
```c
if (fabsf(d->motor.acceleration) < 10) {
    d->traction_control = false;
}
if (timer_older(&d->time, d->wheelslip_timer, 0.2)) {
    if (d->motor.duty_raw < 0.85) {
        d->traction_control = false;
        d->state.wheelslip = false;
    }
}
```

Three hardcoded exit conditions:
- Acceleration drops below 10
- 200ms timeout after last high-duty event
- Duty raw below 0.85

Same weight-dependency reasoning as detection thresholds.

---

### 9. ATR Speed-Dependent EMA Filter Coefficients

**Location**: `src/atr.c:83-91`
```c
if (motor->abs_erpm > 2000)      → 0.9 / 0.1  (fast filter)
else if (motor->abs_erpm > 1000) → 0.95 / 0.05 (medium filter)
else if (motor->abs_erpm > 250)  → 0.98 / 0.02 (slow filter)
else                              → 0 (disabled)
```

These ERPM breakpoints and filter coefficients control how responsive ATR is at
different speeds.

**Why it matters**: The ERPM thresholds assume a particular wheel size and gear
ratio. The filter coefficients control ATR's noise vs. responsiveness tradeoff.

---

### 10. ATR Speed Boost Onset — 3000 ERPM

**Location**: `src/atr.c:101`
```c
if (motor->abs_erpm > 3000 && !motor->braking) {
```

And response boost thresholds at 2500 and 6000 ERPM (`atr.c:125-129`):
```c
if (motor->abs_erpm > 2500) {
    response_boost = config->atr_response_boost;
}
if (motor->abs_erpm > 6000) {
    response_boost *= config->atr_response_boost;
}
```

These assume specific wheel/motor parameters for "low", "medium", and "high"
speed.

---

### 11. ATR Target Smoothing — 0.95/0.05

**Location**: `src/atr.c:120`
```c
atr->target = atr->target * 0.95 + 0.05 * new_atr_target;
```

Controls how fast the ATR target responds to changes. Affects overall ATR
responsiveness.

---

### 12. Booster Speed Stiffness Onset — 3000 ERPM / 10000 Range

**Location**: `src/booster.c:49-51`
```c
const int boost_min_erpm = 3000;
float speedstiffness = fminf(1, (md->abs_erpm - boost_min_erpm) / 10000);
```

Above 3000 ERPM, booster strength ramps up linearly, reaching full effect at
13000 ERPM. These thresholds assume a particular wheel/motor setup.

---

## Tier 3: Subtle Effects — Lower Priority

### 13. Accelerometer Confidence Decay — 0.02

**Location**: `src/balance_filter.c:47-48`
```c
// Hard-coded accelerometer confidence decay of 0.02
float confidence = 1.0 - (0.02 * sqrtf(fabsf(data->acc_mag - 1.0f)));
```

Controls how much the balance filter trusts accelerometers during
non-gravitational acceleration. Acknowledged as hardcoded in the comment.

---

### 14. Brake Tilt Downhill Damper Thresholds

**Location**: `src/brake_tilt.c:60-67`
```c
if ((motor->erpm > 1000 && atr->accel_diff < -1) ||
    (motor->erpm < -1000 && atr->accel_diff > 1)) {
    downhill_damper += fabsf(atr->accel_diff) / 2;
}
if (downhill_damper > 2) {
    bt->target = 0;  // steep downhills disable brake tilt entirely
}
```

The -1/+1 accel_diff threshold and the downhill_damper > 2 cutoff are fixed.
The 1000 ERPM threshold is also hardcoded.

---

### 15. Turn Tilt Yaw Change Filter — 0.8/0.2, Clamp ±0.10

**Location**: `src/turn_tilt.c:62`
```c
tt->yaw_change = 0.8 * tt->yaw_change + 0.2 * clampf(new_change, -0.10, 0.10);
```

The filter coefficient (0.8/0.2) and the clamp range (±0.10 degrees/cycle)
control turn tilt sensitivity and noise rejection.

---

### 16. Turn Tilt Dead Zone — 0.04 deg/cycle

**Location**: `src/turn_tilt.c:71, 86`
```c
if (tt->abs_yaw_change > 0.04 && !unchanged) {
```

Minimum yaw rate before turn tilt starts accumulating. Filters out vibration
and micro-movements.

---

### 17. Winddown Rates — 0.995 / 0.99

**Location**: `src/atr.c:201-202`, `src/brake_tilt.c:90-91`, `src/torque_tilt.c:79`, `src/turn_tilt.c:130`
```c
atr->setpoint *= 0.995;
atr->target *= 0.99;
```

How fast tilt adjustments decay during wheelslip. All modules use the same
hardcoded values. At 800Hz:
- 0.995 → ~138 cycles to halve (~170ms)
- 0.99  → ~69 cycles to halve (~86ms)

---

### 18. Soft Start Ramp Factor — 100

**Location**: `src/main.c:190`
```c
d->softstart_ramp_step_size = (float) 100 / d->float_conf.hertz;
```

Controls how fast Rate-P and Booster ramp in after engaging. At 800Hz, full
effect in ~8 cycles (~10ms). The `100` divisor is fixed.

---

### 19. Brake Tilt Factor Formula Constants — 0.5 / 5.0

**Location**: `src/brake_tilt.c:41`
```c
bt->factor = -(0.5f + (20 - config->braketilt_strength) / 5.0f);
```

The 0.5 baseline and 5.0 divisor shape the strength curve. With
`braketilt_strength` at its max (20), factor = -0.5. At 0, factor = -4.5.

---

### 20. ATR Wind-Down Direction Thresholds — ±3

**Location**: `src/atr.c:155, 182`
```c
if (atr->target > -3 && atr->setpoint > atr->target) {
    // ATR winding down (normal wind down case)
```

Threshold distinguishing "winding down from uphill" vs. "transitioning to
downhill". Affects how step sizes are chosen during ATR direction changes.

---

### 21. ATR Transition Boost Margin — 2 degrees

**Location**: `src/atr.c:136`
```c
const float TT_BOOST_MARGIN = 2;
```

Minimum gap between setpoint and target before transition boost kicks in.
Prevents boost on small corrections.

---

## Summary: Top Candidates for New Config Parameters

| Priority | Constant | Location | Value | Why Tune |
|----------|----------|----------|-------|----------|
| **1** | ATR torque offset | `atr.c:55` | 8A | Weight-dependent rolling resistance |
| **2** | Output current smoothing | `main.c:946` | 0.8/0.2 | Core ride feel, weight-sensitive |
| **3** | Wheelslip accel threshold | `main.c:546` | 15 | Weight/tire/terrain dependent |
| **4** | ATR torque linearity breakpoint | `atr.c:68` | 25A | Motor/battery dependent |
| **5** | PID brake transition rate | `pid.c:51` | 0.01/0.99 | Ride feel preference |
| **6** | ATR target smoothing | `atr.c:120` | 0.95/0.05 | ATR responsiveness |
| **7** | Acc confidence decay | `balance_filter.c:48` | 0.02 | Filter tuning |
| **8** | Winddown rates | multiple | 0.995 | Wheelslip recovery feel |

The **8A torque offset** is the single most impactful hardcoded value for rider
weight. It is also the one the developer explicitly flagged as a placeholder.
Combined with `atr_amps_accel_ratio` / `atr_amps_decel_ratio` (which *are*
already tunable), making the torque offset configurable would give the ATR
system proper weight-adaptive behavior.
