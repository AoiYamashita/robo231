import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    receiver = launch_ros.actions.Node(
            package = 'robo231',
            executable = 'receiver',
            output = 'screen'
            )
    
    usb_cam = launch_ros.actions.Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            #parameters = [{'contrast':32,'brightness':32,'saturation':32}],
            )
    
    return launch.LaunchDescription([receiver,usb_cam])
