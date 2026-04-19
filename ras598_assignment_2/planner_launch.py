import sys
import os
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_root      = get_package_share_directory("ras598_assignment_2")
    map_yaml_path = os.path.join(pkg_root, "map.yaml")
    scout_path    = os.path.join(pkg_root, "grading_scout.py")
    rviz_path     = os.path.join(pkg_root, "planning.rviz")

    return LaunchDescription([

        # 1. Stage simulator
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "stage_ros2", "demo.launch.py", "world:=cave", "use_stamped_velocity:=false",
            ],
            env={**os.environ, "QT_QPA_PLATFORM": "wayland"},
            output="screen",
            name="stage_simulator",
        ),

        # 2. map_server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[{
                "yaml_filename": map_yaml_path,
                "use_sim_time": False,
            }],
            output="screen",
        ),

        # 3. Lifecycle manager
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),

        # 4. Grading scout
        ExecuteProcess(
            cmd=["python3", scout_path],
            output="screen",
            name="grading_scout",
        ),

        # 5. RViz
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["rviz2", "-d", rviz_path],
                    output="screen",
                    name="rviz2",
                )
            ],
        ),

        # 6. Planner node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="ras598_assignment_2",
                    executable="planner_node",
                    name="planner_node",
                    output="screen",
                )
            ],
        ),
    ])


def main():
    """Allows running as: python3 planner_launch.py"""
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    print("--- Starting ROS 2 Launch Service ---")
    return ls.run()


if __name__ == "__main__":
    sys.exit(main())
