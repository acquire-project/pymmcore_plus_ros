#!/usr/bin/env python3

from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
from useq import MDAEvent, MDASequence
from pymmcore_plus import CMMCorePlus

import os
os.environ["MMCORE_PLUS_SIGNALS_BACKEND"] = "psygnal"

class PyMMCorePlusNode(Node):
    def __init__(self):
        super().__init__('py_mmcore_plus_node')

        try:
            from image_transport_py import ImageTransport
            self.image_transport = ImageTransport('imagetransport_pub', image_transport='compressed')
            self.img_pub = self.image_transport.advertise('image_raw', 10)
        except:
            self.get_logger().warning("python image transport not found, falling back on publishing sensor_msgs:msg:Image topic")
            self.img_pub = self.create_publisher(Image, 'image_raw', 10)

        self.core = CMMCorePlus.instance()
        
        self.declare_my_parameters()
        self.configure()
        self.activate()
    
    def declare_my_parameters(self):
        
        # path to hardware system config file
        self.declare_parameter(
            "system_config_path", 
            "",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to system configuration file, typically generated by MicroManager. Defaults to demo config if left empty'
            )
        )
        
        # path to MDA sequence file (yaml)
        self.declare_parameter(
            "mda_sequence_path", 
            "",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Path to MDA sequence. See https://pymmcore-plus.github.io/useq-schema/api/ '
            )
        )
        
    def configure(self):
        
        # read system config path param, and initialize the core with it.
        system_config_path = self.get_parameter('system_config_path').get_parameter_value().string_value
        if(len(system_config_path) == 0):
            # load demo config if blank
            self.core.loadSystemConfiguration() 
        else:
            self.core.loadSystemConfiguration(system_config_path) 
        
        # Turn off hardware sequencing, it's not working correctly right now
        self.core.mda.engine.use_hardware_sequencing = False
        
        
        # see https://pymmcore-plus.github.io/useq-schema/api/ 
        mda_sequence_path = self.get_parameter('mda_sequence_path').get_parameter_value().string_value
        self.sequence = MDASequence.from_file(mda_sequence_path)
    
    def activate(self):
        self.core.mda.events.frameReady.connect(self.on_new_frame)
        # run the sequence in a separate thread 
        self.core.run_mda(self.sequence)
        

    def on_new_frame(self, data: np.ndarray, event: MDAEvent):
        
        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera'
        if data.dtype == np.uint8:
            image_msg.encoding = 'mono8'
        elif data.dtype == np.uint16:
            image_msg.encoding = 'mono16'
        else:
            raise TypeError("Only uint8 & uint16 types are supported.")
            
        image_msg.height = data.shape[0]
        image_msg.width = data.shape[1]
        image_msg.step = data.strides[0]
        image_msg.data = data.tobytes()
        self.img_pub.publish(image_msg)
        self.get_logger().info('Publishing image')


def main(args=None):
    rclpy.init(args=args)
    node = PyMMCorePlusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
