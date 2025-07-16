#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
import math
import numpy as np

class CameraFOVSimulator(Node):
    def __init__(self):
        super().__init__('camera_fov_simulator')
        
        # Setup TF
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_world_tf()
        
        # Parameters
        self.sp_reso = 0.4 # mm
        self.pixel_size = 3.45 # μm
        self.camera_height = 5.5 #!

        self.table_length = 1.5
        self.table_width = 0.7
        self.hfov = 2.0*np.arctan2(0.5*self.table_length, self.camera_height)
        self.vfov = 2.0*np.arctan2(0.5*self.table_width, self.camera_height)
        H = int(self.table_length/(self.sp_reso/1000.0))
        V = int(self.table_width/(self.sp_reso/1000.0))

        print(f"Camera height: {self.camera_height} m")
        print(f"Table length: {self.table_length} m")
        print(f"Table width: {self.table_width} m")
        print(f"")
        
        print(f"Error: {self.sp_reso} mm")
        print(f"HFOV: {np.rad2deg(self.hfov)}")
        print(f"VFOV: {np.rad2deg(self.vfov)}")
        print(f"H: {H} pixel")
        print(f"V: {V} pixel")
        resolution_standard = self.determine_resolution_standard(
                                    h_resolution=H, 
                                    v_resolution=V
        )
        print(f"Pixel Size: {self.pixel_size} μm")
        print(f"Resolution Standard: {resolution_standard}")
        print(f"Focal length H: {( (self.pixel_size*H) / (2.0*np.tan(self.hfov/2)) )/1000.0} mm")
        print(f"Focal length V: {( (self.pixel_size*V) / (2.0*np.tan(self.vfov/2)) )/1000.0} mm")
        
        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'camera_fov_markers', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)
        
        self.get_logger().info("Camera FOV Simulator started")
    
    def publish_world_tf(self):
        """Publish static world->map transform"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"  # Parent frame
        transform.child_frame_id = "world" # Child frame
        transform.transform.rotation.w = 1.0  # Identity quaternion
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published world->map transform")
    
    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        
        # Clear previous (use DELETEALL only once per namespace)
        clear_marker = Marker()
        clear_marker.header.frame_id = "world"
        clear_marker.header.stamp = now
        clear_marker.ns = "fov_simulation"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add markers
        marker_array.markers.append(self.create_table_marker(now))
        marker_array.markers.append(self.create_camera_marker(now))
        marker_array.markers.extend(self.create_fov_markers(now))
        
        self.marker_pub.publish(marker_array)
    
    def create_table_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "table"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.pose.position.z = -0.01
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.table_length
        marker.scale.y = self.table_width
        marker.scale.z = 0.02
        marker.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.8)
        return marker
    
    def create_camera_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = stamp
        marker.ns = "camera"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.pose.position.z = self.camera_height
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        return marker
    
    def create_fov_markers(self, stamp):
        h_half = math.tan(self.hfov/2) * self.camera_height
        v_half = math.tan(self.vfov/2) * self.camera_height
        
        # Line marker
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = stamp
        line_marker.ns = "fov_lines"
        line_marker.id = 2
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.01  # Line thickness
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        
        camera_pt = Point(z=self.camera_height)
        corners = [
            Point(x=h_half, y=v_half),
            Point(x=h_half, y=-v_half),
            Point(x=-h_half, y=-v_half),
            Point(x=-h_half, y=v_half)
        ]
        
        # Lines from camera to corners
        for corner in corners:
            line_marker.points.append(camera_pt)
            line_marker.points.append(corner)
        
        # Lines between corners
        for i in range(4):
            line_marker.points.append(corners[i])
            line_marker.points.append(corners[(i+1)%4])
        
        return [line_marker]
    

    def determine_resolution_standard(self, h_resolution, v_resolution):
        """
        Improved resolution standard detector that better handles widescreen formats
        and provides more accurate classification for resolutions near standard formats.
        """
        # Resolution standards with common aliases
        standards = [
            {'name': '12K', 'width': 12288, 'height': 6480, 'tolerance': 0.05},
            {'name': '8K FUHD', 'width': 7680, 'height': 4320, 'tolerance': 0.05},
            {'name': '8K DCI', 'width': 8192, 'height': 4320, 'tolerance': 0.05},
            {'name': '5K', 'width': 5120, 'height': 2880, 'tolerance': 0.05},
            {'name': '4K UHD', 'width': 3840, 'height': 2160, 'tolerance': 0.10},  # Wider tolerance for 4K
            {'name': '4K DCI', 'width': 4096, 'height': 2160, 'tolerance': 0.10},
            {'name': '2.5K', 'width': 2560, 'height': 1440, 'tolerance': 0.10},
            {'name': '2K DCI', 'width': 2048, 'height': 1080, 'tolerance': 0.10},
            {'name': 'Full HD', 'width': 1920, 'height': 1080, 'tolerance': 0.10},
            {'name': 'HD', 'width': 1280, 'height': 720, 'tolerance': 0.10}
        ]
        
        # Calculate aspect ratio
        aspect = h_resolution / v_resolution
        
        # Check against each standard with tolerance
        for std in standards:
            std_aspect = std['width'] / std['height']
            
            # Check if resolution is within tolerance (percentage of standard)
            h_match = abs(h_resolution - std['width']) <= std['width'] * std['tolerance']
            v_match = abs(v_resolution - std['height']) <= std['height'] * std['tolerance']
            aspect_match = abs(aspect - std_aspect) < 0.1  # 10% aspect ratio tolerance
            
            # For widescreen formats, prioritize width matching
            if h_match and aspect_match:
                return std['name']
        
        # Special case handling for common near-4K resolutions
        if 3500 <= h_resolution <= 4200 and 1700 <= v_resolution <= 2300:
            if aspect > 1.77:  # Wider than 16:9
                return "4K (Ultra Wide)"
            else:
                return "4K (Near Standard)"
        
        # Fallback to megapixel-based classification
        megapixels = (h_resolution * v_resolution) / 1_000_000
        
        if megapixels >= 30:
            return f"8K-class (≈{megapixels:.1f}MP)"
        elif megapixels >= 7:
            return f"4K-class (≈{megapixels:.1f}MP)"
        elif megapixels >= 2:
            return f"Full HD-class (≈{megapixels:.1f}MP)"
        else:
            return f"HD-class (≈{megapixels:.1f}MP)"
    




def main(args=None):
    rclpy.init(args=args)
    node = CameraFOVSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()