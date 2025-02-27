import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        #launch_ros.actions.Node(
            #package='visionPiRos2',
            #executable='object_detection_node',
            #name='object_detection'
        #),
        launch_ros.actions.Node(
            package='visionPiRos2',
            executable='gui',
            name='vision_gui'
        )
    ])
