# Tractor Image Rectifier

A ROS2 package for image rectification that supports both compressed and uncompressed images with configurable QoS profiles.

## Features

- Supports both compressed and uncompressed image topics
- Automatic detection of image format based on topic name
- Configurable QoS profiles (Reliable or Best Effort)
- Flexible topic remapping
- Maintains image format consistency between input and output

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select tractor_image_rectifier
source install/setup.bash
```

## Usage

### Basic Usage

The default configuration uses compressed images with best effort QoS:

```bash
ros2 launch tractor_image_rectifier image_rectifier.launch.py
```

### Configuration Options

#### QoS Profile

To use reliable QoS:
```bash
ros2 launch tractor_image_rectifier image_rectifier.launch.py reliable_qos:=true
```

#### Image Format

For uncompressed images:
```bash
ros2 launch tractor_image_rectifier image_rectifier.launch.py input_topic:=input/image_raw
```

#### Custom Topics

To use custom topic names:
```bash
ros2 launch tractor_image_rectifier image_rectifier.launch.py \
    input_topic:=/my_camera/image_raw \
    output_topic:=/my_camera/image_rect
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `reliable_qos` | bool | false | Use reliable QoS when true, best effort when false |
| `input_topic` | string | 'input/image_raw/compressed' | Input image topic (add /compressed for compressed images) |
| `output_topic` | string | 'output/image_rect' | Base output topic (automatically adds /compressed if input is compressed) |

### Topic Mapping

The node automatically handles the following topic mappings:

- Input image: `input_topic` (compressed or uncompressed)
- Camera info: `input/camera_info`
- Output image: `output_topic` (maintains same format as input)

## Dependencies

- ROS2 (tested on Humble)
- OpenCV
- cv_bridge
- image_geometry

## License

[Add your license here]

## Contributing

[Add contribution guidelines here] 