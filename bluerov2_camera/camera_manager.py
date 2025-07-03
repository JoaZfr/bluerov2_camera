import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/frontal', 10)
        
        # Initialize GStreamer
        Gst.init(None)
        
        pipeline_desc = (
            "udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! "
            "video/x-raw,width=640,height=360,format=RGB ! tee name=t "
            "t. ! queue ! appsink name=appsink emit-signals=true max-buffers=1 drop=true "
                )

        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.on_new_sample, None)
        
        # Start the pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Initialize GMainLoop
        self.loop = GObject.MainLoop()
        
        # Create a timer to periodically call the GMainLoop iteration
        # 10ms period (100Hz) is typically sufficient for GStreamer processing
        self.timer = self.create_timer(0.1, self.gstreamer_callback)
            
    def gstreamer_callback(self):
        # Process GStreamer events in the ROS2 timer callback
        if self.loop.is_running():
            # Run one iteration of the main loop non-blocking
            self.loop.get_context().iteration(False)
            
    def on_new_sample(self, sink, data):
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
            
            # Read raw data
            result, mapinfo = buf.map(Gst.MapFlags.READ)
            if result:
                # Convert buffer to numpy array
                data = np.frombuffer(mapinfo.data, dtype=np.uint8)
                expected_size = width * height * 3  # BGR format
                if len(data) == expected_size:
                    data = data.reshape((height, width, 3))  # Reshape to BGR
                else:
                    self.get_logger().warn(f"Unexpected data size {len(data)}, expected {expected_size}. Skipping frame.")
                    buf.unmap(mapinfo)
                    return Gst.FlowReturn.OK
                
                # Create ROS2 Image message
                img_msg = Image()
                img_msg.header.frame_id = 'camera_frame'
                img_msg.height = height
                img_msg.width = width
                img_msg.encoding = 'bgr8'  # Changed to bgr8 to match the format
                img_msg.is_bigendian = False
                img_msg.step = width * 3  # 3 bytes per pixel (BGR)
                img_msg.data = data.tobytes()

                # Publish the image
                self.publisher_.publish(img_msg)
                
                buf.unmap(mapinfo)
                
        return Gst.FlowReturn.OK
    
    def destroy_node(self):
        # Cleanup GStreamer pipeline and main loop
        self.loop.quit()
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping...")
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()