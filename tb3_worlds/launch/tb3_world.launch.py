from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory("tb3_worlds")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    tb3_world_dir = get_package_share_directory("tb3_worlds")
    default_map = join(tb3_world_dir, "maps", "sim_house_map.yaml")
    default_world = join(tb3_world_dir, "worlds", "sim_house.world")

    # Start Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': f'-v 4 {default_world}'}.items(),
    )

    # Start robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Spawn the turtlebot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_gazebo_dir, "launch", "spawn_waffle_pi.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription(
        [
            gzserver_cmd,
            robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
        ]
    )
