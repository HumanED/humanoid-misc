"""
Test for moving multiple CAN BLDC motors simultaneously per cycle.
Make sure that all motors' positions are set to 0 before fitting them into their mounts and that
they are fitted so that their output shafts are touching their minimum points of rotation.
Position units are in revolutions. A position of 1 is a full 360 rotation.

The assigned IDs are as follows:
    1. Hip
    2. Thigh
    3. Knee
    4. Foot pitch

Based off 'move_to.py' in moteus examples.
"""

import argparse
import asyncio
import moteus

def check_id_duplicates(args):
    ids = {args.id1, args.id2, args.id3, args.id4}
    ids.sort()

    try:
        for i in range(1, 3):
            if ids[i] == ids[i - 1]:
                raise DupeError
        break
    except DupeError:
        Print("Duplicate ID in command")

async def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('--id1', default = 1, type = int, help = 'ID of hip motor (1)')
    parser.add_argument('--id2', default = 2, type = int, help = 'ID of thigh motor (2)')
    parser.add_argument('--id3', default = 3, type = int, help = 'ID of knee motor (3)')
    parser.add_argument('--id4', default = 4, type = int, help = 'ID of foot pitch motor (4)')

    parser.add_argument('--period', type = float, default = 5.0, help = 'Period in seconds for full cycle (default: 5.0)')

    args = parser.parse_args()

    # raises exception if duplicate motor ID values are found
    check_id_duplicates(args)

    transport = moteus.get_singleton_transport(args)

    hip = moteus.Controller(id = args.id1, transport = transport)
    thigh = moteus.Controller(id = args.id2, transport = transport)
    knee = moteus.Controller(id = args.id3, transport = transport)
    foot_pitch = moteus.Controller(id = args.id4, transport = transport)

    # send stop commands
    await transport.cycle([hip.make_stop(), thigh.make_stop(), knee.make_stop(), foot_pitch.make_stop()])
    

    # TODO: find positions for each motor
    # define positions for each motor
    positions_a = {hip: 0.0, thigh: 0.0, knee: 0.0, foot_pitch: 0.0}
    positions_b = {hip:, thigh:, knee:, foot_pitch:}

    move_duration = args.period / 2.0

    # test start
    print("test begin")
    print()

    try:
        while True:
            # move to 1st position
            # TODO: replace '...' with relevant data
            await moteus.move_to([...], duration = move_duration)
    
            # move to 2nd position
            await moteus.move_to([...], duration = move_duration)

    except KeyboardInterrupt:
        print("\Stopping test")
    finally:
        await transport.cycle([hip.make_stop(), thigh.make_stop(), knee.make_stop(), foot_pitch.make_stop()])

if __name__ == 'main':
    asyncio.run(main())
