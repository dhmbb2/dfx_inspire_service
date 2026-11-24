# Changelog

## 1.0.1

+ Initial release.

## 1.2

- support unitree g1
## 1.3 (dhmbb2 fork)

- G1 driver rewritten (`inspire_g1.cpp`, `include/rh56/`): complete-frame serial exchanges with
  deadlines, input flush and checked write acknowledgements; FTDI low-latency mode; one thread per
  hand; hands that do not answer are skipped; ports and IDs as options; `q < 0` leaves a finger
  unchanged; speed and force threshold settable; force calibration at register 1009; force,
  current, temperature, status, error and the age of `q` in the state message.
- `test/fake_rh56.py`: fake hand on a pseudo-terminal for testing without hardware.
- H1 driver unchanged.
