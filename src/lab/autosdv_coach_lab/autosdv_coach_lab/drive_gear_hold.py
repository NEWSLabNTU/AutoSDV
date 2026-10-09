"""drive_gear_hold: publish GearCommand DRIVE at a fixed rate. Simulator only.

Autoware's shift_decider commands PARK while /autoware/state is
WAITING_FOR_ROUTE (park_on_goal: true), and the coach lab never has a route,
so the simulated vehicle would never leave PARK. The lab control component
already sets park_on_goal: false, which makes shift_decider echo the reported
gear instead; this node makes DRIVE certain regardless of the simulator's
initial gear.

It must be the ONLY publisher on its topic. coach_pursuit_sim.launch.xml
moves vehicle_cmd_gate's gear output off /control/command/gear_cmd
(output_gear_cmd) and points this node at it; never publish alongside
shift_decider on /control/shift_decider/gear_cmd, where the two would fight.

The real AutoSDV actuator ignores gear; never launch this on the car.
"""

import rclpy
from autoware_vehicle_msgs.msg import GearCommand
from rclpy.node import Node


class DriveGearHold(Node):
    def __init__(self):
        super().__init__('drive_gear_hold')
        self.declare_parameter('output_topic', '/control/command/gear_cmd')
        self.declare_parameter('rate', 10.0)
        self.pub = self.create_publisher(
            GearCommand, self.get_parameter('output_topic').value, 1)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def tick(self):
        m = GearCommand()
        m.stamp = self.get_clock().now().to_msg()
        m.command = GearCommand.DRIVE
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DriveGearHold()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
