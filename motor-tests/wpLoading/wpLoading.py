'''
Prototype program to synchronously control multiple servos using ruckig for trajectory generation and loading position waypoints from user terminal input
'''

import argparse
import asyncio
import math
from ruckig import Ruckig, InputParameter, Trajectory, Result
import time

import moteus

TWO_PI = 2.0 * math.pi

# down angles in revolutions
DOWN_HIP = 0.0
DOWN_KNEE = 0.0

# torques of limbs
K_LEG = 0.9
K_SHIN = 0.1

# directions
KNEE_DIR = 1

def parse_args():
    parser = argparse.ArgumentParser(
        description='Synchronized multi-joint motion using ruckig')
    parser.add_argument(
        '-t', '--targets', type=str, default='1,2',
        help='Comma-separated list of servo IDs (default: 1,2)')
    parser.add_argument(
        '--max-velocity', type=float, default=0.5,
        help='Maximum velocity in rev/s (default: 0.5)')
    parser.add_argument(
        '--max-accel', type=float, default=2.0,
        help='Maximum acceleration in rev/s^2 (default: 2.0)')
    parser.add_argument(
        '--max-jerk', type=float, default=10.0,
        help='Maximum jerk in rev/s^3 (default: 10.0)')
    parser.add_argument(
        '--cycle-time', type=float, default=0.01,
        help='Cycle time in seconds (default: 0.01)')
    parser.add_argument(
        '--base-amplitude', type=float, default=0.25,
        help='Base motion amplitude in revolutions (default: 0.25)')
    moteus.make_transport_args(parser)
    return parser.parse_args()

class SynchronisedMotion:
    def __init__(self, args):
        self.args = args
        self.servo_ids = [int(x.strip()) for x in args.targets.split(',')]
        self.num_dofs = len(self.servo_ids)

        self.transport = moteus.get_singleton_transport(args)
        self.controllers = {
            servo_id: moteus.Controller(id=servo_id, transport=self.transport)
            for servo_id in self.servo_ids
        }

        self.otg = Ruckig(self.num_dofs)
        self.inp = InputParameter(self.num_dofs)
        self.trajectory = Trajectory(self.num_dofs)

        self.initial_positions = None
        self.waypoints = []
        self.current_waypoint = 0
        self.segment_start_time = 0.0
        self.last_print = 0.0

    def link_angles(self, positions_revs):
        '''Motor positions in revolutions to radians'''
        angles = []
        acc = 0.0
        for sign, pos in zip(self.signs, positions_revs):
            acc += sign * pos
            phis.append(TWO_PI * acc)
        return phis

    def gravity_ff(self, hip, knee):
        ''' this stuffd'''
        thigh = 2 * math.pi * (hip - DOWN_HIP)
        shin = thigh + KNEE_DIR * 2 * math.pi * (knee - DOWN_KNEE)

        return [(K_LEG - K_SHIN) * math.sin(thigh) + K_SHIN * math.sin(shin), K_SHIN * math.sin(shin)]

    async def _query_initial_positions(self):
        print("Stopping servos and querying initial positions")

        await self.transport.cycle([
            c.make_stop() for c in self.controllers.values()
            ])
        await asyncio.sleep(0.1)
        print("stopped")
        results = await self.transport.cycle([
            c.make_query() for c in self.controllers.values()
            ])
        print("results")

        self.initial_positions = []
        for servo_id in self.servo_ids:
            pos = 0.0
            for result in results:
                if result.id == servo_id:
                    pos = result.values.get(moteus.Register.POSITION, 0.0)
                    break
            self.initial_positions.append(pos)
            print(f"  Servo {servo_id}: position = {pos:.4f}")

    def _init_trajectory_state(self):
        self.inp.current_position = list(self.initial_positions)
        self.inp.current_velocity = [0.0] * self.num_dofs
        self.inp.current_acceleration = [0.0] * self.num_dofs

    async def initialise(self):
        # set kinematic limits
        self.inp.max_velocity = [self.args.max_velocity] * self.num_dofs
        self.inp.max_acceleration = [self.args.max_accel] * self.num_dofs
        self.inp.max_jerk = [self.args.max_jerk] * self.num_dofs

        await self._query_initial_positions()

        self._init_trajectory_state()

    def update_trajectory(self, target_positions):
        """Sample current trajectory at the actual elapsed wall-clock time.
        Advances to the next waypoint when the current segment is done.
        Returns (positions, velocities) or (None, None) on error."""
        self.inp.target_position = target_positions
        self.inp.target_velocity = [0.0] * self.num_dofs

        result = self.otg.calculate(self.inp, self.trajectory)
        if result not in (Result.Working, Result.Finished):
            print(f"Trajectory update failed: {result}")


    async def send_transport(self):
        start_time = time.time()
        
        # stream trajectory until arrival
        while True:
            elapsed = time.time() - start_time
            if elapsed >= self.trajectory.duration:

                # get final motor states
                end_pos, end_vel, end_acc = self.trajectory.at_time(
                    self.trajectory.duration)
                self.inp.current_position = list(end_pos)
                self.inp.current_velocity = list(end_vel)
                self.inp.current_acceleration = list(end_acc)
                break

            positions, velocities, _ = self.trajectory.at_time(elapsed)
            ff_torques = self.gravity_ff(positions[0], 0)
            print(f"{positions}, {ff_torques}")
            commands = [
                self.controllers[servo_id].make_position(
                    position=positions[i],
                    velocity=velocities[i],
                    feedforward_torque=ff_torques[i],
                    query=True,
                )
                for i, servo_id in enumerate(self.servo_ids)
            ]
            results = await self.transport.cycle(commands)

            # print telemetry at 5hz
            TELEM_PERIOD = 0.2
            now = time.time()

            if now - self.last_print > 0.2:
                self.last_print = now
                parts = [f"ID{r.id}: p={r.values.get(moteus.Register.POSITION, 0.0):+.3f}"
                         for r in results]
                print(" | ".join(parts), end="\r")

            await asyncio.sleep(self.args.cycle_time)

    async def hold_loop(self):
        # stream commands for motors to hold current position to satisfy watchdog timeout rate
        ff_torques = self.gravity_ff(self.inp.current_position[0], 0)
        while True:
            commands = [
                    self.controllers[s_id].make_position(
                        position=self.inp.current_position[i],
                        velocity=0.0,
                        feedforward_torque=ff_torques[i],
                        query=False
                    )
                    for i, s_id in enumerate(self.servo_ids)
            ]
            await self.transport.cycle(commands)
            await asyncio.sleep(0.02)   # 50Hz holds

    async def run(self):
        # running loop
        loop = asyncio.get_running_loop()
        hold_task = asyncio.create_task(self.hold_loop())

        # get positions from user terminal input
        try:
            while True:
                prompt = f"\nEnter positions"
                user_inp = await loop.run_in_executor(None, input, prompt)

                try:
                    target = [float(x.strip()) for x in user_inp.split(',')]
                    if len(target) != self.num_dofs:
                        print(f"Invalid number of positions, expected {self.num_dofs} and got {len(target)}")
                except ValueError:
                    print("Invalid numbers entered")
    
                # pause hold background task to send desired motor trajectories
                hold_task.cancel()
                try:
                    await hold_task
                except asyncio.CancelledError:
                    pass

                self.update_trajectory(target)
                await self.send_transport()

                # resume holding position at new target
                hold_task = asyncio.create_task(self.hold_loop())

        finally:
            hold_task.cancel()
            await self.transport.cycle([c.make_stop() for c in self.controllers.values()])
            print("\nMotors stopped")

async def main():
    args = parse_args()
    motion = SynchronisedMotion(args)
    await motion.initialise()
    await motion.run()

if __name__ == "__main__":
    asyncio.run(main())
