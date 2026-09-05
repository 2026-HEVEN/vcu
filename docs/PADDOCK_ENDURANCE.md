# Paddock endurance / dynamometer profile

This branch extends the existing Cluster paddock request into a conservative
one-hour dynamometer envelope. It does **not** make 300 A/motor continuous.
Full pedal in paddock mode is intentionally limited by all of the following:

- 30 A maximum phase-current command per motor;
- linear speed taper from 8 km/h to zero positive drive at 10 km/h;
- the greater of front-wheel vehicle speed and motor-RPM-derived driven-wheel
  speed, so a rear-wheel-only roller cannot bypass the speed limit;
- 3.5 kW controller-reported input-power ceiling;
- 90 A sum of absolute controller bus currents;
- 70 A absolute BMS pack-current ceiling;
- valid BMS data and valid controller/motor temperature telemetry;
- the existing controller and motor thermal derating;
- a 60-minute active timer, latched at zero output until paddock mode is
  switched off and re-armed with released throttle below the entry speed.

## Why full pedal is still low output

The Bexel pack is approximately 4.14 kWh. The 2026-09-05 road logs reached
about 8.47 kW and 153 A around 40--45 km/h at 300 A/motor, which cannot be
maintained for one hour from this pack. Paddock mode therefore interprets
100% pedal as 100% of the endurance envelope, not 100% of the normal 300 A
peak envelope.

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
4. Run 5 minutes, stop and inspect coolant, motors, controllers, cables and
   connectors before extending to 15, 30 and 60 minutes.
5. Treat any CAN fault, stale feedback, `sensorBlock=1`, `currentLimit=1`,
   `pwr=1`, `therm=1`, abnormal noise, smell, vibration or leakage as a stop
   condition requiring inspection.

## Serial diagnostics

The 1 Hz summary includes:

```text
LIMIT pad=... timeout=... sensorBlock=... currentLimit=...
      pwr=... therm=... scale=... speed=...kmh
      P=measured/estimatedW BMS=... V=... I=... T=...
```

The 30 A value remains an initial endurance setting. It must not be raised
until a short staged run confirms motor temperature, coolant performance,
controller current scale, BMS current and the dynamometer load point.
