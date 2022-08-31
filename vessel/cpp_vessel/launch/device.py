import launch
import launch_ros.actions

def generate_launch_description():
  return launch.LaunchDescription([
    launch_ros.actions.Node(
      package='base_address_technologies_japan_drivers',
      executable='motor_cpp',
      namespace='cpp_vessel',
      name='motor',
      output='screen',
      parameters=[
        { 'network/device_ip':'192.168.39.13' },
        { 'network/device_ip_mask':'255.255.255.0' },
        { 'network/my_port':50000 },
        { 'network/device_port':50003 },
        { 'topicname/sub/motor_control':'motor_control' },
        { 'topicname/sub/propeller_control':'prop_control' }
      ]
    )
  ])