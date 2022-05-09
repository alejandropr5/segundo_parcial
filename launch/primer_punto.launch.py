import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node_exe'
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'default_cam']
        ),
        launch_ros.actions.Node(
            package='segundo_parcial',
            executable='img_sub_and_process',
            name='img_sub_and_process'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/alejandro/colcon_ws/src/segundo_parcial/rviz_config/primer_punto.rviz']
        )
    ])