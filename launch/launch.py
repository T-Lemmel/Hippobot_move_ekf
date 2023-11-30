from simple_launch import SimpleLauncher
import os

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('rviz', True)
base_path = os.path.abspath(os.path.dirname(__file__))

def launch_setup():


    # transfert GPS to actual poses
    sl.node('move_ekf', 'stamp_wamv',
            output='screen')

    # run an EKF for wamv
    sl.node('robot_localization', 'ekf_node', name = 'ekf',
            parameters = [sl.find('move_ekf', 'ekf.yaml')],
            namespace = 'wamv',
            remappings = {'odometry/filtered': 'odom'})
    
    sl.node('move_ekf', 'waypoint_publisher')
    sl.node('move_ekf', 'place_frame.py')
    sl.node('move_ekf', 'tracker.py')

    if sl.arg('rviz'):
        sl.rviz(base_path + '/config.rviz')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
