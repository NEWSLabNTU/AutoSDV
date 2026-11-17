"""Stop the launch with a named reason.

Included where a launch argument asks for something this build cannot run, so the
user reads why instead of getting a stack that silently lacks it:

    <include file="$(find-pkg-share autosdv_launch)/launch/util/refuse.launch.py"
             if="$(var some_unsupported_switch)">
      <arg name="reason" value="..."/>
    </include>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _refuse(context):
    raise RuntimeError('AutoSDV: ' + LaunchConfiguration('reason').perform(context))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reason', description='why the launch cannot go on'),
        OpaqueFunction(function=_refuse),
    ])
