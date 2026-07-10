"""Launches the full pipeline: camera -> perception -> control (+ visualizer).

    ros2 launch lane_pipeline pipeline.launch.py video_path:=/abs/path/to/video.mp4

Optional arguments mirror the standalone script's CLI flags:

    kp:=1.5 smoothing:=0.15 max_missed_frames:=5
    show_preview:=false output_path:=/abs/path/to/output.mp4 loop:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    video_path = LaunchConfiguration('video_path')
    loop = LaunchConfiguration('loop')
    smoothing = LaunchConfiguration('smoothing')
    max_missed_frames = LaunchConfiguration('max_missed_frames')
    kp = LaunchConfiguration('kp')
    show_preview = LaunchConfiguration('show_preview')
    output_path = LaunchConfiguration('output_path')

    camera_node = Node(
        package='lane_pipeline',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'video_path': ParameterValue(video_path, value_type=str),
            'loop': loop,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            description='Absolute path to the input video file'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Restart the video from the beginning when it ends'),
        DeclareLaunchArgument(
            'smoothing', default_value='0.2',
            description='Lane EMA smoothing factor, 0-1; lower = smoother but laggier'),
        DeclareLaunchArgument(
            'max_missed_frames', default_value='5',
            description='Frames to keep reusing the last known lane before giving up'),
        DeclareLaunchArgument(
            'kp', default_value='1.0',
            description='Steering gain applied to the computed heading angle'),
        DeclareLaunchArgument(
            'show_preview', default_value='true',
            description='Show the live preview window'),
        DeclareLaunchArgument(
            'output_path', default_value='',
            description='Record the annotated stream to this mp4 path (empty = no recording)'),

        camera_node,
        # camera_node exits once the clip ends (and isn't looping); tear
        # down the rest of the pipeline so recording nodes get a chance to
        # finalize their output instead of being left running forever.
        RegisterEventHandler(OnProcessExit(
            target_action=camera_node,
            on_exit=[EmitEvent(event=Shutdown(reason='camera_node finished'))],
        )),
        Node(
            package='lane_pipeline',
            executable='perception_node',
            name='perception_node',
            parameters=[{
                'smoothing': smoothing,
                'max_missed_frames': max_missed_frames,
            }],
        ),
        Node(
            package='lane_pipeline',
            executable='control_node',
            name='control_node',
            parameters=[{'kp': kp}],
        ),
        Node(
            package='lane_pipeline',
            executable='visualizer_node',
            name='visualizer_node',
            parameters=[{
                'show_preview': show_preview,
                'output_path': ParameterValue(output_path, value_type=str),
            }],
        ),
    ])
