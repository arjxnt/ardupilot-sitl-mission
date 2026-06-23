# ArduPilot SITL Search & Orbit

A pymavlink script that flies a simulated ArduCopter through an expanding-square
search pattern, detects a hardcoded target geofence, breaks off to orbit it,
then RTLs and lands. Runs against ArduPilot's SITL simulator.

ArduPilot's `waf` build doesn't run natively on Windows, so you'll need
WSL2/Ubuntu or another Linux box.

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

This starts MAVProxy and streams MAVLink on `udp:127.0.0.1:14550`, which is
what the script connects to. Leave it running.

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

Arms, takes off to 30m in GUIDED, flies the search pattern, and if it gets
within `TARGET_RADIUS_M` of the target it breaks off, orbits at 15m, and
logs the detection with a timestamp. Then RTL and land.

To force a quick detection, shrink `LEG_UNIT_M` so the pattern sweeps closer
to `TARGET_LAT/LON`, or move the target onto the generated path.

All the tunable params (altitude, search area, target coords, orbit radius)
are constants at the top of `search_and_orbit.py`.

## Safety

Simulation only. Don't point this at a real flight controller without a
real RC override and proper safety review — there's no failsafe logic here
beyond what SITL itself provides.
