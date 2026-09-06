# Paddock speed-dependent current test profile

This branch turns the existing Cluster paddock request into a continuous
speed-dependent phase-current test envelope. It does **not** make 300 A/motor
continuous. Full pedal in paddock mode is limited by all of the following:

- a phase-current ceiling that falls linearly from 300 A/motor at 0 km/h to
  50 A/motor at 80 km/h, and stays at 50 A/motor above 80 km/h;
- a 600 A/s propulsion rise limit, so 0 to 300 A takes 0.5 seconds;
- the greater of front-wheel vehicle speed and motor-RPM-derived driven-wheel
  speed, so a low or missing WSS reading cannot bypass the current envelope;
- an 8.0 kW measured/estimated input-power ceiling;
- 200 A sum of absolute controller bus currents;
- 150 A absolute BMS pack-current ceiling;
- valid BMS data and valid controller/motor temperature telemetry;
- the existing controller and motor thermal derating;

The current equation is:

```text
ratio = clamp(speed / 80 km/h, 0, 1)
phase_current_limit_each = 300 A + (50 A - 300 A) * ratio
```

## Why phase current falls with speed

The Bexel pack is approximately 4.14 kWh. The 2026-09-05 road logs reached
about 8.47 kW and 153 A around 40--45 km/h at 300 A/motor, which cannot be
maintained as speed continues to rise. The falling phase-current ceiling gives
stronger launch current while reducing the electrical demand as motor speed
rises. The independent power, bus-current and pack-current scalers can reduce
the command below the linear ceiling.

## Cooling prerequisites

The vehicle's EZkontrol controllers use an antifreeze liquid-cooling circuit.
Before an endurance run, verify coolant level, pump flow, equal flow through
both controllers, radiator fan operation, hose routing, air bleeding, and that
both controller temperature values respond to load. Motor cooling is a
separate limit and must be verified independently.

## Required pre-test sequence

1. Lift/roller test with zero throttle and confirm both controller and motor
   temperatures are real values, not the EZkontrol `-40 C` invalid sentinel.
2. Confirm `BMS=1` in the `LIMIT` serial line before enabling paddock mode.
3. Select paddock mode only below 3 km/h with released throttle.
4. Start with the driven wheels lifted or a controlled dynamometer load. Check
   the 0/20/40/60/80 km/h current points before any road test.
5. Treat any CAN fault, stale feedback, `sensorBlock=1`, `currentLimit=1`,
   `pwr=1`, `therm=1`, abnormal noise, smell, vibration or leakage as a stop
   condition requiring inspection.

## Serial diagnostics

The 1 Hz summary includes:

```text
LIMIT pad=... sensorBlock=... currentLimit=...
      pwr=... therm=... scale=... speed=...kmh Ilim=...A
      P=measured/estimatedW BMS=... V=... I=... T=...
```

`Ilim` is the calculated per-motor phase-current ceiling before the independent
bus-current, pack-current, power and thermal scalers. The 300 A zero-speed value
is a short test ceiling, not a manufacturer-confirmed continuous rating and not
proof that the installed cooling system can sustain it.

The rise limiter only delays an increase in propulsion-current magnitude.
Throttle release, a lower speed/current/power/thermal ceiling, stale CAN
feedback, controller faults and invalid required telemetry reduce the command
immediately. Reverse propulsion uses the same magnitude ramp; regenerative
braking is not delayed by this launch ramp. The `LIMIT` line reports `slew=1`
while the rise limiter is actively below the requested current.
