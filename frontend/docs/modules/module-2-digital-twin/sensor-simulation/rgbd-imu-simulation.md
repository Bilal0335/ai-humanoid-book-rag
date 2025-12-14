---
sidebar_position: 2
---

# RGB-D and IMU Simulation for Humanoid Perception

## 2.2 RGB-D & Stereo Depth Cameras

RGB-D cameras provide crucial visual and depth information for humanoid robotics perception systems. These sensors enable object recognition, scene understanding, and spatial awareness capabilities essential for autonomous operation.

### Camera Parameters: Intrinsics, Extrinsics, Distortion

RGB-D camera simulation requires accurate modeling of optical properties:

**Intrinsic Parameters**:
- **Focal Length**: Determines field of view and magnification
  - Horizontal and vertical focal lengths (fx, fy) in pixels
  - Typically ranges from 300-1000 pixels for standard cameras
- **Principal Point**: Optical center coordinates (cx, cy)
  - Usually at image center for undistorted cameras
  - Critical for accurate 3D reconstruction
- **Image Dimensions**: Resolution in pixels (width × height)
  - Common resolutions: 640×480, 1280×720, 1920×1080

**Extrinsic Parameters**:
- **Position**: 3D coordinates relative to robot base frame
- **Orientation**: Rotation relative to robot coordinate system
- **Mounting Configuration**: Multiple cameras for stereo or panoramic views
- **Coordinate Frame**: Consistent with ROS tf2 transform system

**Distortion Models**:
- **Radial Distortion**: Barrel or pincushion effects (k1, k2, k3)
- **Tangential Distortion**: Manufacturing imperfections (p1, p2)
- **Thin Prism Distortion**: Higher-order effects (k4, k5, k6)

### Depth Accuracy, Field of View, Resolution Settings

RGB-D camera performance characteristics:

**Depth Range and Accuracy**:
- **Range**: Typically 0.3m to 5-10m for structured light cameras
- **Accuracy**: ±1-5mm at 1m distance, degrading with range
- **Resolution**: 240×320 to 1024×1024 pixels
- **Update Rate**: 15-30 Hz for real-time applications

**Field of View Considerations**:
- **Horizontal FOV**: 57°-87° for standard cameras
- **Vertical FOV**: 43°-65° corresponding to horizontal FOV
- **Diagonal FOV**: 70°-110° for wide-angle applications
- **Stereo Baseline**: Distance between stereo camera pairs (typically 6-12cm)

**Resolution and Quality Settings**:
- **Color Resolution**: 640×480 to 1920×1080
- **Depth Resolution**: Often different from color resolution
- **Bit Depth**: 8-bit, 16-bit, or higher for depth data
- **Compression**: Lossless or lossy compression options

### Point Cloud Generation from Depth Images

Converting depth images to 3D point clouds for perception tasks:

**Depth-to-3D Conversion Process**:
1. **Pixel-to-3D Mapping**: Using camera intrinsic parameters
2. **Coordinate System Transformation**: Converting to robot frame
3. **Point Cloud Assembly**: Combining multiple frames over time
4. **Filtering and Processing**: Removing noise and outliers

**ROS 2 Integration**:
- `sensor_msgs/PointCloud2` message format
- `depth_image_proc` package for depth processing
- TF transforms for coordinate alignment
- Real-time point cloud streaming

### Integration with Perception Tasks

RGB-D cameras enable various perception capabilities:

**Object Recognition**:
- 3D object detection and classification
- Instance segmentation with depth information
- Pose estimation using 3D-2D correspondences

**Scene Understanding**:
- Semantic segmentation with depth context
- 3D scene reconstruction and mapping
- Spatial relationship analysis

**Navigation and Mapping**:
- Visual-inertial odometry (VIO)
- Structure from motion (SfM)
- 3D occupancy grid mapping

### Example: Using Depth Data for Simple Obstacle Detection

Implementation example for obstacle detection using RGB-D data:

```xml
<!-- RGB-D camera configuration in URDF -->
<gazebo reference="rgbd_camera_link">
  <sensor type="depth" name="humanoid_rgbd_camera">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera_name>rgbd_camera</camera_name>
      <image_topic_name>/image_raw</image_topic_name>
      <depth_image_topic_name>/depth/image_raw</depth_image_topic_name>
      <point_cloud_topic_name>/depth/points</point_cloud_topic_name>
      <frame_name>rgbd_camera_optical_frame</frame_name>
      <point_cloud_cutoff>0.1</point_cloud_cutoff>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
      <Cx_prime>0</Cx_prime>
      <Cx>320.5</Cx>
      <Cy>240.5</Cy>
      <focal_length>320</focal_length>
    </plugin>
  </sensor>
</gazebo>
```

## 2.3 IMU Simulation

Inertial Measurement Units (IMUs) provide crucial orientation and motion data for humanoid robots, enabling balance control, navigation, and state estimation.

### Accelerometer, Gyroscope, Orientation

IMU sensors measure different aspects of robot motion:

**Accelerometer**:
- **Measurement**: Linear acceleration along 3 axes (x, y, z)
- **Range**: Typically ±2g to ±16g (where g = 9.81 m/s²)
- **Resolution**: 12-24 bits, translating to micro-g sensitivity
- **Applications**: Gravity detection, impact sensing, vibration analysis

**Gyroscope**:
- **Measurement**: Angular velocity around 3 axes (roll, pitch, yaw)
- **Range**: Typically ±250°/s to ±2000°/s
- **Resolution**: 16-20 bits, translating to 0.008-0.061 °/s/LSB
- **Applications**: Rotation detection, orientation tracking, motion control

**Orientation**:
- **Measurement**: 3D orientation (quaternions, Euler angles)
- **Integration**: Combining accelerometer and gyroscope data
- **Reference**: Typically Earth's gravity and magnetic field
- **Accuracy**: Dependent on sensor fusion algorithms

### Realistic Noise Models (Bias, Drift, Sampling Rate)

Accurate IMU simulation requires realistic noise modeling:

**Bias**:
- **Constant Bias**: Systematic offset in measurements
- **Temperature Dependence**: Bias variation with temperature
- **Initialization**: Bias estimation during calibration
- **Modeling**: Additive constant to true measurements

**Drift**:
- **Random Walk**: Low-frequency bias variations
- **Time-dependent**: Accumulated errors over time
- **ARW (Angle Random Walk)**: For gyroscope angular errors
- **VRW (Velocity Random Walk)**: For accelerometer velocity errors

**Sampling Rate Effects**:
- **Aliasing**: High-frequency signals appearing as low-frequency
- **Quantization**: Discrete measurement levels
- **Jitter**: Timing variations in measurement acquisition
- **Bandwidth**: Effective frequency response limitations

### How IMU Integrates with Locomotion and Balancing

IMU data is critical for humanoid robot stability:

**Balance Control**:
- **Zero Moment Point (ZMP)**: Foot placement optimization
- **Inverted Pendulum Model**: Balance control algorithms
- **Attitude Control**: Maintaining upright posture
- **Fall Detection**: Emergency response triggers

**Locomotion Integration**:
- **Gait Analysis**: Step detection and timing
- **Orientation Control**: Maintaining heading during movement
- **Terrain Adaptation**: Adjusting gait based on inclination
- **Motion Planning**: Incorporating IMU feedback in control loops

### Validation and Testing

**Checking Sensor Data Fidelity vs Expected Values**:
- **Static Tests**: Verify gravity vector measurement
- **Dynamic Tests**: Validate response to known motions
- **Temperature Tests**: Assess bias and drift characteristics
- **Long-term Tests**: Evaluate drift accumulation over time

**Using ros2 topic echo, rviz2, Unity Visualization**:
- Real-time monitoring of IMU data streams
- Visualization of orientation and acceleration
- Integration with 3D simulation environments
- Cross-validation with other sensors

**Reproducibility Checks**:
- Consistent behavior across multiple simulation runs
- Deterministic noise generation with fixed seeds
- Synchronized data acquisition timing
- Repeatable test scenarios

## Implementation Example

Here's an example of IMU configuration in Gazebo for a humanoid robot:

```xml
<!-- IMU sensor configuration in URDF -->
<gazebo reference="imu_link">
  <sensor name="humanoid_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev> <!-- ~0.1 deg/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0017</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev> <!-- ~0.0017g -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

This comprehensive sensor simulation framework enables realistic perception system development for humanoid robots in digital twin environments.