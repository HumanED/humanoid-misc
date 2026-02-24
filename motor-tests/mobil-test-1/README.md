Test for moving multiple CAN BLDC motors simultaneously per cycle.

## Purpose
- Simple test to move 4 leg motors between position 0 and their respective maximum positions periodically.
- Once the test is run successfully, the motors will move from their current position to the other every cycle.
- The length of the cycle period can be determined by the user.

> [!WARNING]
> Make sure that all motors' positions are set to 0 before fitting them into their mounts and that they are fitted so that their output shafts are touching their minimum points of rotation.
> Position units are in revolutions. A position of 1 is a full 360 rotation.

## Executing test
- The test is executed as a a commandline program. Make sure you're in the `mobile-test` directory or wherever it is you've copied the source code.
- The command format is shown below:
```
python3 main.py --id1 N1 --id2 N2 --id3 N3 --id4 N4 --period N5
```
- Where `N1-N4` are motor IDs (integers) and `N5` is the cycle period in seconds (float).
- If run without any of the parameters, the program will be executed with the default arguments below:
    - id1 to id4: 1 to 4
    - period: 5 seconds

## Motor IDS
The current leg motor ID designations are as follows:
    1. Hip
    2. Thigh
    3. Knee
    4. Foot pitch

Based off [move_to.py](https://github.com/mjbots/moteus/blob/main/lib/python/examples/move_to.py) in Moteus examples.

