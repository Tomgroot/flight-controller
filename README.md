# flight-controller

A Rust flight controller for an STM32F401 quad (see [PARTS.md](PARTS.md)), tested against a native software-in-the-loop (SITL) simulator first.

```
crates/
  fc-core/   no_std flight code, runs on the F401 and in SITL   <- you implement this
  sim/       quad physics + MPU-6050 + scripted RC models
  sitl/      lockstep loop: sim -> fc-core -> sim, logs to Rerun
```

## Setup (macOS and Ubuntu)

1. Install Rust through [rustup](https://rustup.rs). `rust-toolchain.toml` picks the toolchain and the `thumbv7em-none-eabihf` target automatically.
   ```sh
   brew uninstall rust   # macOS only, if installed: it shadows rustup and is too old for rerun (needs rustc ≥ 1.96)
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source "$HOME/.cargo/env"   # or open a new shell
   ```
2. Install the Rerun viewer. Its version must match `rerun` in `Cargo.toml`:
   ```sh
   cargo install rerun-cli@0.38.1 --locked
   ```

## Run

```sh
cargo run -p sitl                         # simulate 10 s, opens the Rerun viewer
cargo run -p sitl -- --realtime           # run at wall-clock speed
cargo run -p sitl -- --save out.rrd       # headless, open later with `rerun out.rrd`
cargo test                                # sim physics tests
cargo build -p fc-core --target thumbv7em-none-eabihf   # check fc-core stays no_std
```

Without a local toolchain you can use Docker:

```sh
docker run --rm -v "$PWD":/src -w /src -e CARGO_TARGET_DIR=/tmp/target rust:1 \
  cargo run -p sitl -- --save /src/out.rrd
```

## Implementing the flight code

Everything happens in `FlightController::update` in [crates/fc-core/src/lib.rs](crates/fc-core/src/lib.rs). It runs at 1 kHz with:

- `ImuSample`: body-frame (x forward, y left, z up) gyro in rad/s and specific force in m/s². Reads +9.81 on z at rest. Includes noise and bias.
- `RcChannels`: CRSF values 172–1811 in AETR order. Channel 4 is the arm switch.
- It returns `MotorOutputs`: 0.0–1.0 in Betaflight quad-X order (0 rear-right CW, 1 front-right CCW, 2 rear-left CCW, 3 front-left CW).

It currently returns zeros, so the quad sits on the ground. The scripted transmitter (`crates/sim/src/rc.rs`) arms at 1 s and ramps throttle to 50 % between 2 and 4 s. Hover is at roughly 54 % motor output.

## Tuning the model

Airframe parameters live in `QuadParams::default()` in [crates/sim/src/quad.rs](crates/sim/src/quad.rs), and sensor noise and bias in `ImuParams::default()` in [crates/sim/src/imu.rs](crates/sim/src/imu.rs). Mass, inertia and thrust are estimates; replace them with measurements from the real build.

## Roadmap

- Stage 2: Gazebo Harmonic as an alternative backend, a headless server in Docker that `sitl` talks to over UDP. `fc-core` stays unchanged.
- Firmware crate for the STM32F401 calling the same `FlightController::update`.
