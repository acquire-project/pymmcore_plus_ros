import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    my_share_path = FindPackageShare('pymmcore_plus_ros')
    
    image_width = LaunchConfiguration('image_width')
    image_width_launch_arg = DeclareLaunchArgument('image_width', default_value='640', description='Image width')
    
    image_height = LaunchConfiguration('image_height')
    image_height_launch_arg = DeclareLaunchArgument('image_height', default_value='480', description='Image height')
    
    system_config_path = LaunchConfiguration('system_config_path')
    system_config_path_launch_arg = DeclareLaunchArgument(
        'system_config_path', 
        default_value = PathJoinSubstitution([my_share_path, 'config', 'Automaton_noLC_Blackfly_TriggerScope.cfg']),
        description='Micromanager system configuration file path'
    )
    
    mda_sequence_path = LaunchConfiguration('mda_sequence_path')
    mda_sequence_path_launch_arg = DeclareLaunchArgument(
        'mda_sequence_path', 
        default_value = PathJoinSubstitution([my_share_path, 'config', 'mda_seq_cz.yml']),
        description='MDA sequence file path (useq yaml file)'
    )

    zarr_out_path = LaunchConfiguration('zarr_out_path')
    zarr_out_path_launch_arg = DeclareLaunchArgument('zarr_out_path', default_value='/tmp/out.zarr', description='zarr output path')
    
    pymmcore_plus_node = Node(
        package='pymmcore_plus_ros', 
        executable='pymmcore_plus_node', 
        parameters=[{
            'system_config_path': system_config_path,
            'mda_sequence_path': mda_sequence_path,
            'image_width': image_width, 
            'image_height': image_height,
        }],
        output='screen')

    zarr_writer_node = Node(
        package='acquire_zarr', 
        executable='image_zarr_writer_node', 
        parameters=[{
            'zarr_path': zarr_out_path,
            'dimension_sizes': [0, image_height, image_width],
            'chunk_sizes': [1, image_height, image_width],
        }],
        output='screen')

    # List of nodes, arguements, etc to add to the launch description.
    return launch.LaunchDescription([
        image_width_launch_arg,
        image_height_launch_arg,
        system_config_path_launch_arg,
        mda_sequence_path_launch_arg,
        zarr_out_path_launch_arg,
        pymmcore_plus_node,
        #zarr_writer_node,
        ])

