
Training Plan (WIP)

Goal: learn a stable altitude hold controller in PX4 SITL via PPO.

Phase A (done)

Wire PX4↔MAVSDK↔Gym env

Run PPO for 20k steps, verify reward trending up, save model

Phase B (next)

Curriculum: randomize initial z in [0.5, 3.0] m

Shape reward: -|alt_err| - 0.02|vz_cmd| + 0.5 * I(|alt_err|<0.15)

Terminate if |alt_err|>4 m or crash

Phase C

Domain randomization: IMU noise, mass, drag, wind

Longer horizon, eval episodes, checkpointing

Metrics

Mean |alt_err|, variance, time-in-band (±10/20/30 cm)

Episode success rate (no terminations)

TODO: fill in target ranges and acceptance criteria.
