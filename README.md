# ArduPilot SITL Search & Orbit

A Python script that autonomously flies a *simulated* drone through a search
pattern, recognizes when it's found a target, and circles it.

## What this actually does

- **SITL** (Software In The Loop) is ArduPilot's drone simulator — it runs a
  full virtual flight controller on your computer so you can test autonomous
  flight without a real drone.
- **pymavlink** is a Python library for talking to that flight controller
  using **MAVLink**, the messaging protocol drones use (telemetry in,
  commands out).
- The script connects to the simulator, takes off, and flies an
  **expanding-square search pattern** — a real search-and-rescue technique
  where the drone flies in an outward-growing square spiral to cover ground
  systematically.
- It has a hardcoded "target" location (just GPS coordinates) with a radius
  around it. While searching, the script keeps checking the drone's current
  position against that target.
- The moment the drone gets close enough to the target, it abandons the
  search pattern and flies in a circle ("orbit") around it — like a drone
  would do to keep a camera trained on something it found — and prints out
  the target's coordinates with a timestamp, as if logging a detection.
- Once that's done, it sends the drone home and lands.

This is a simulation/demo project, not a connection to a real aircraft.

## Before you start

ArduPilot's build tool (`waf`) doesn't work natively on Windows, so you need
a Linux environment. The easiest way on Windows is **WSL2** (Windows
Subsystem for Linux), which lets you run a real Ubuntu Linux install inside
Windows.

## Setup

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

## Run SITL

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --vehicle ArduCopter --console --map
```

This starts MAVProxy (a ground-control relay tool) and streams MAVLink data
on `udp:127.0.0.1:14550` — a local network address on your own machine. The
script connects to that address to send/receive commands. Leave this
terminal running the whole time; it's your simulated drone.

Default home is CMAC (-35.363261, 149.165230). If you start SITL with
`--custom-location`, update `SEARCH_CENTER_LAT/LON` and `TARGET_LAT/LON` in
the script to match.

## Run the mission

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 search_and_orbit.py
```

What happens, in order:
1. **Arms** the drone (enables the motors — a safety step drones require
   before they'll fly) and switches to **GUIDED mode** (lets a script
   command the drone directly, instead of a human pilot or a pre-set
   mission).
2. Takes off to 30m and starts flying the search pattern.
3. If it gets within `TARGET_RADIUS_M` of the target coordinates, it stops
   searching, orbits the target at 15m, and prints a timestamped log entry
   with the target's exact GPS position.
4. Sends an **RTL** (Return To Launch) command, which tells the drone to fly
   back to its takeoff point and land itself automatically.

To force a quick detection, shrink `LEG_UNIT_M` so the pattern sweeps closer
to `TARGET_LAT/LON`, or move the target onto the generated path.

All the tunable params (altitude, search area, target coords, orbit radius)
are constants at the top of `search_and_orbit.py`.

## Safety

Simulation only. Don't point this at a real flight controller without a
real RC override and proper safety review — there's no failsafe logic here
beyond what SITL itself provides.
