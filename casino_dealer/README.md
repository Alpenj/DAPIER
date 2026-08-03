# DAPIER Casino Dealer / CardBench

This package establishes the simulator-independent contract for a future
dual-arm casino dealer. It deliberately starts below ROS hardware adapters,
Isaac Lab, LeRobot, and learned policies so every later integration shares one
testable definition of observations, actions, tasks, and deal order.

## What works now

- A versioned CardBench v0 observation/action contract
- Two 4-DOF arm states and targets
- Independent left/right vacuum state and control channels
- Required overhead vision with optional wrist cameras
- A deterministic blackjack opening deal for one to seven players
- Explicit bimanual roles: left arm stabilizes the deck; right arm moves cards
- Dependency-free unit tests and JSON output

## Ubuntu CLI quick start

Python 3.10 or newer is sufficient for the planner and tests. On a new
education PC, configure GitHub authentication first if the DAPIER repository
is private, then run:

```bash
git clone https://github.com/Alpenj/DAPIER.git
cd DAPIER/casino_dealer
python3 -m unittest discover -s test -v
python3 -m casino_dealer.cli --players 3
```

To update an existing checkout later:

```bash
cd DAPIER
git pull --ff-only origin main
cd casino_dealer
python3 -m casino_dealer.cli --players 3 --compact
```

Generate a three-player opening deal:

```bash
cd casino_dealer
python -m casino_dealer.cli --players 3
```

Run the tests without ROS 2:

```bash
cd casino_dealer
python -m unittest discover -s test -v
```

Build as a ROS 2 Jazzy package:

```bash
cd ~/jdcobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select casino_dealer
source install/setup.bash
casino_blackjack_plan --players 3
```

## Stable contract

The packaged contract is
`casino_dealer/contracts/cardbench_v0.json`. Its key rule is that
`observation.state.*.joint_position` means a measured joint position. A driver
must not label its last command as measured feedback. This distinction matters
for replay, imitation learning, and sim-to-real evaluation.

The v0 action vector contains ten scalar channels:

```text
left joint targets   4
right joint targets  4
left vacuum command  1
right vacuum command 1
```

The low-level action contract uses absolute joint targets. A future GR00T
adapter may derive relative action deltas without changing the hardware-facing
contract. Vacuum commands are normalized `float32` channels in the range
0.0 to 1.0 so ACT and VLA policies can consume one continuous action vector;
the hardware adapter is responsible for converting that value to pump control.

## Current boundary

This package does not actuate the existing arm and does not claim that the
current hardware is bimanual. It has no card perception, collision simulation,
vacuum driver, success detector, teleoperation recorder, or learned policy yet.

Recommended next milestones:

1. Add measured joint feedback and separate command/state interfaces.
2. Prototype one vacuum end effector and an overhead camera.
3. Add a single-card pick/place task with objective success detection.
4. Record CardBench episodes in LeRobot Dataset format.
5. Train an ACT baseline before comparing SmolVLA and GR00T.
6. Duplicate the calibrated arm and add the bimanual simulator adapter.
