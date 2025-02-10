#!/usr/bin/env python3

from image_transport_py import ImageTransport
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

import numpy as np
from useq import MDAEvent, MDASequence
from pymmcore_plus import CMMCorePlus

import os
os.environ["MMCORE_PLUS_SIGNALS_BACKEND"] = "psygnal"

class PyMMCorePlusNode(Node):
    def __init__(self):
        super().__init__('py_mmcore_plus_node')

        self.image_transport = ImageTransport(
            'imagetransport_pub', image_transport='compressed'
        )
        self.img_pub = self.image_transport.advertise('camera/image', 10)

        self.core = CMMCorePlus.instance()
        
        self.configure()
        self.activate()
        
    def configure(self):
        
        self.core.loadSystemConfiguration()  #  load demo configuration 
        
         # see https://pymmcore-plus.github.io/useq-schema/api/ 
        self.sequence = MDASequence(
            channels=["DAPI", {"config": "FITC", "exposure": 50}],
            time_plan={"interval": 2, "loops": 5},
            z_plan={"range": 4, "step": 0.5},
            axis_order="tpcz",
        )
    
    def activate(self):
        self.core.mda.events.frameReady.connect(self.on_new_frame)
        # run the sequence in a separate thread 
        self.core.run_mda(self.sequence)
        

    def on_new_frame(self, data: np.ndarray, event: MDAEvent):
        
        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera'
        image_msg.encoding = 'mono8'
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
