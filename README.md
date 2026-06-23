# ArduPilot Search & Orbit Mission System

An autonomous drone mission system: it flies a search pattern, recognizes
when it's found a target, breaks off to orbit it, and returns home. It
runs two ways — against a real ArduPilot SITL simulator over MAVLink, or
instantly against a built-in lightweight flight simulator with zero setup.

## What this actually does

- **SITL** (Software In The Loop) is ArduPilot's drone simulator — a full
  virtual flight controller you run on your own computer to test autonomous
  flight without a real drone.
- **pymavlink** talks to that flight controller using **MAVLink**, the
  messaging protocol real drones use (telemetry in, commands out).
- The mission flies an **expanding-square search pattern** — a real
  search-and-rescue technique where the drone sweeps an outward-growing
  square spiral to cover ground systematically — plus a **lawnmower
  pattern** for covering a known rectangular area evenly.
- It has a target location (GPS coordinates) with a detection radius. While
  searching, it continuously checks its current position against that
  target.
- The moment it's within range, it abandons the search pattern and orbits
  the target at a fixed radius — like a drone holding a camera on something
  it found — and logs a timestamped detection with the GPS fix.
- Then it returns home and lands.

## Why this is a "system" and not just a script

The mission logic (search → detect → orbit → RTL) is written once, against
a small `Vehicle` interface (`missionlib/vehicle.py`), and never talks to
MAVLink directly. Two things implement that interface:

- `missionlib/backends/mavlink_backend.py` — the real thing, flies SITL
  over MAVLink.
- `missionlib/backends/sim_backend.py` — a small kinematic simulator with
  no external dependencies. It moves a point toward its current target at a
  fixed speed and reports position, which is enough to exercise the entire
  mission end-to-end.

Because the mission code doesn't care which one it's talking to, the same
logic that flies real SITL is fully covered by an automated test suite
(`tests/test_mission_sim.py`) that runs in seconds with no simulator
installed. That test suite is what caught the actual bug in an earlier
version of this project: the hardcoded target coordinates didn't sit on the
search path, so the detection logic never fired. It's fixed now, and the
test (`test_target_on_path_is_detected_and_orbited`) is what guarantees it
stays fixed.

## Try it right now (no SITL needed)

```bash
pip install -r requirements.txt -r requirements-dev.txt
python run_mission.py --backend sim --plot flight_path.png
```

This runs the full mission against the built-in simulator in a couple of
seconds, prints the detection log, and saves a plot of the search pattern,
the detection point, and the orbit:

![flight path](flight_path.png)

Run the test suite the same way:

```bash
pytest tests/ -v
```

## Run it against real SITL

ArduPilot's build tool (`waf`) doesn't run natively on Windows, so you need
a Linux environment — WSL2/Ubuntu works.

```powershell
wsl --install -d Ubuntu
```

```bash
sudo apt update && sudo apt install -y git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

./waf configure --board sitl
./waf copter
```

Launch the simulator:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --vehicle ArduCopter --console --map
```

This starts MAVProxy, which streams MAVLink on `udp:127.0.0.1:14550`. Leave
it running — it's your simulated drone. Default home is CMAC
(-35.363261, 149.165230); if you start SITL elsewhere with
`--custom-location`, update the coordinates in `run_mission.py` to match.

In a second terminal:

```bash
pip install -r requirements.txt
python run_mission.py --backend mavlink
```

## What happens during a run

1. **Arms** the drone and switches to **GUIDED mode** (lets the script
   command it directly instead of a human pilot).
2. Takes off to 30m and starts the search pattern.
3. Once within range of the target, breaks off, orbits it at 15m, and logs
   the detection with a timestamp and GPS fix.
4. Sends **RTL** (Return To Launch) and waits for it to land and disarm.

## Project layout

```
missionlib/
  geo.py            lat/lon <-> meters math
  patterns.py        expanding-square and lawnmower search patterns
  vehicle.py          the backend-agnostic interface
  mission.py          the actual search/detect/orbit/RTL state machine
  plotting.py          flight-path plot generation
  backends/
    sim_backend.py    in-memory simulator, no dependencies
    mavlink_backend.py   real MAVLink/SITL implementation
tests/                pytest suite, runs against the sim backend
run_mission.py         CLI entrypoint
```

## Safety

The MAVLink backend is for simulation only. Don't point it at a real flight
controller without a real RC override and proper safety review — none of
the logic here is a substitute for a vehicle's own configured failsafes.
