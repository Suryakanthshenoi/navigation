# DirectionalLayer Plugin for ROS Costmap_2d

A costmap plugin that stores 8-bit directional preferences per grid cell, enabling path planners to consider preferred movement directions for different areas of the map.

## Overview

The DirectionalLayer plugin extends the ROS costmap_2d framework by adding directional constraints to grid cells. Each cell stores an 8-bit mask indicating which movement directions are preferred or allowed in that location.

## Features

- ✅ **8-bit Directional Masks**: Each grid cell stores directional preferences using bit flags
- ✅ **Zone-based Configuration**: Define directional zones using polygons
- ✅ **Runtime Zone Management**: Add zones dynamically via ROS services
- ✅ **YAML Configuration**: Load predefined zones from configuration files
- ✅ **RViz Visualization**: View zones and directional arrows in RViz
- ✅ **Thread-safe Access**: Concurrent access to mask grid with proper locking
- ✅ **Pluginlib Integration**: Standard ROS plugin architecture

## Directional Encoding

The 8-bit mask encodes movement directions as follows:

| Direction | Bit | Mask Value | Binary | Arrow Color | Description |
|-----------|-----|------------|--------|-------------|-------------|
| **East** | 0 | `1` | `0b00000001` | 🔴 Red | Point right (→) |
| **Northeast** | 1 | `2` | `0b00000010` | 🔵 Blue | Point diagonal up-right (↗) |
| **North** | 2 | `4` | `0b00000100` | 🟢 Green | Point up (↑) |
| **Northwest** | 3 | `8` | `0b00001000` | 🔵 Blue | Point diagonal up-left (↖) |
| **West** | 4 | `16` | `0b00010000` | 🔴 Red | Point left (←) |
| **Southwest** | 5 | `32` | `0b00100000` | 🔵 Blue | Point diagonal down-left (↙) |
| **South** | 6 | `64` | `0b01000000` | 🟢 Green | Point down (↓) |
| **Southeast** | 7 | `128` | `0b10000000` | 🔵 Blue | Point diagonal down-right (↘) |

### Common Mask Values

| Description | Mask Value | Binary | Directions |
|-------------|------------|--------|------------|
| **All directions** | `255` | `0b11111111` | All 8 directions |
| **No movement** | `0` | `0b00000000` | Blocked |
| **East only** | `1` | `0b00000001` | East (→) |
| **North only** | `4` | `0b00000100` | North (↑) |
| **West only** | `16` | `0b00010000` | West (←) |
| **South only** | `64` | `0b01000000` | South (↓) |
| **Horizontal only** | `17` | `0b00010001` | East + West (← →) |
| **Vertical only** | `68` | `0b01000100` | North + South (↑ ↓) |
| **Northeast quadrant** | `15` | `0b00001111` | E, NE, N, NW |
| **Custom example** | `6` | `0b00000110` | Northeast + North (↗ ↑) |

## Installation

1. **Build the plugin**:
   ```bash
   cd your_catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **Add to costmap configuration**:
   ```yaml
   plugins:
     - {name: directional_layer, type: "costmap_2d::DirectionalLayer"}
   
   directional_layer:
     enabled: true
     default_mask: 255
     zones_file: ""
     visualization_topic: "directional_layer/visualization"
   ```

## Usage

### Service Interface

Add directional zones using the ROS service:

```bash
rosservice call /costmap/directional_layer/add_zone "
name: 'loading_dock'
mask: 1
polygon:
- {x: 10.0, y: 5.0, z: 0.0}  
- {x: 12.0, y: 5.0, z: 0.0}
- {x: 12.0, y: 7.0, z: 0.0}
- {x: 10.0, y: 7.0, z: 0.0}
"
```

### Python API

```python
import rospy
from costmap_2d.srv import DirectionalZone
from geometry_msgs.msg import Point32

# Wait for service
rospy.wait_for_service('/costmap/directional_layer/add_zone')
add_zone = rospy.ServiceProxy('/costmap/directional_layer/add_zone', DirectionalZone)

# Create zone polygon
polygon = [
    Point32(x=1.0, y=1.0, z=0.0),
    Point32(x=3.0, y=1.0, z=0.0), 
    Point32(x=3.0, y=3.0, z=0.0),
    Point32(x=1.0, y=3.0, z=0.0)
]

# Add zone with north-only movement
response = add_zone(name="restricted_area", mask=1, polygon=polygon)
print(f"Success: {response.success}, Message: {response.message}")
```

### C++ API

```cpp
#include <costmap_2d/directional_layer.h>

// Get directional mask at world coordinates
costmap_2d::DirectionalLayer* dir_layer = /* get layer */;
unsigned char mask = dir_layer->getMask(world_x, world_y);

// Check if direction is allowed
bool north_allowed = mask & 1;        // Bit 0
bool east_allowed = mask & 4;         // Bit 2  
bool south_allowed = mask & 16;       // Bit 4
bool west_allowed = mask & 64;        // Bit 6
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled` | bool | true | Enable/disable the layer |
| `default_mask` | int | 255 | Default mask for undefined areas |
| `zones_file` | string | "" | YAML file with predefined zones |
| `visualization_topic` | string | "visualization" | Topic for RViz markers |

## YAML Zone Configuration

```yaml
zones:
  - name: "loading_dock"
    mask: 1  # North only
    polygon:
      - [10.0, 5.0]
      - [12.0, 5.0] 
      - [12.0, 7.0]
      - [10.0, 7.0]
      
  - name: "corridor"  
    mask: 5  # North + East
    polygon:
      - [15.0, 10.0]
      - [20.0, 10.0]
      - [20.0, 12.0]
      - [15.0, 12.0]
```

## Visualization

The plugin publishes `visualization_msgs/MarkerArray` for RViz:

- **Polygon markers**: Show zone boundaries (colored by mask value)  
- **Arrow markers**: Show allowed directions at zone centers
- **Topic**: `/costmap/directional_layer/visualization`

### RViz Setup

1. Add **MarkerArray** display
2. Set topic to `/costmap/directional_layer/visualization`  
3. View colored polygons and directional arrows

## Testing

Run the demonstration:

```bash
# Terminal 1: Start ROS
roscore

# Terminal 2: Run demo
cd your_catkin_ws/src/costmap_2d
python3 directional_layer_demo.py

# Terminal 3: Test with actual service
python3 test_directional_layer.py
```

## Use Cases

### Warehouse Navigation
- **Loading docks**: Vehicles approach from specific directions
- **Narrow aisles**: Restrict movement to forward/backward only
- **Intersections**: Control traffic flow patterns

### Autonomous Vehicles  
- **One-way roads**: Encode traffic direction rules
- **Roundabouts**: Specify clockwise/counterclockwise flow
- **Parking lots**: Guide approach angles to parking spaces

### Robot Path Planning
- **Corridor navigation**: Maintain wall-following behavior
- **Obstacle avoidance**: Prefer certain directions around obstacles
- **Multi-robot systems**: Reduce conflicts with directional preferences

## Architecture

### Class Structure

```
DirectionalLayer
├── DirectionalZoneData      # Internal zone storage
├── publishVisualization()   # RViz marker generation  
├── addZoneService()        # Service callback
├── getMask()               # Public mask access
└── applyZonesToGrid()      # Apply zones to costmap grid
```

### Thread Safety

- Uses `boost::recursive_mutex` for grid access
- Service calls are thread-safe
- Visualization updates are atomic

## Integration with Path Planners

Path planners can query directional preferences:

```cpp
// In your path planner
costmap_2d::DirectionalLayer* dir_layer = 
    static_cast<costmap_2d::DirectionalLayer*>(
        costmap->getPluginByName("directional_layer"));

if (dir_layer) {
    unsigned char mask = dir_layer->getMask(x, y);
    
    // Check if planned direction is allowed
    int direction_bit = getDirectionBit(planned_direction);
    bool allowed = mask & (1 << direction_bit);
    
    if (!allowed) {
        // Modify path or add cost penalty
    }
}
```

## Troubleshooting

### Plugin Not Loading
- Check `costmap_plugins.xml` registration
- Verify plugin in package.xml exports
- Ensure library is built and linked

### Service Not Available
- Check costmap node is running
- Verify plugin is enabled in configuration
- Check namespace/topic remapping  

### Visualization Not Showing
- Verify MarkerArray topic in RViz
- Check visualization_topic parameter
- Ensure zones have been added

### Performance Issues
- Reduce visualization update frequency
- Use fewer complex polygons
- Optimize zone overlap checks

## Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/new-feature`
3. Commit changes: `git commit -am 'Add new feature'`
4. Push to branch: `git push origin feature/new-feature`  
5. Submit pull request

## License

This project is licensed under the BSD License - see the LICENSE file for details.

## Authors

- **DirectionalLayer Plugin** - ROS1 Costmap_2d extension for directional path planning

## Acknowledgments

- ROS Navigation Stack team for the costmap_2d framework
- Pluginlib architecture for extensible plugin system
- RViz visualization system for debugging support


rosservice call /move_base/global_costmap/directional_zone/add_zone "
name: 'oriagin_dock'
mask: 16
polygon:
  points:
  - x: -1.5
    y: 1.5
    z: 0.0
  - x: 1.5
    y: 1.5
    z: 0.0
  - x: 1.5
    y: 4.5
    z: 0.0
  - x: -1.5
    y: 4.5
    z: 0.0
"
rosservice call /move_base/global_costmap/directional_zone/add_zone "
name: 'origin_dock'
mask: 1
polygon:
  points:
  - x: -1.5
    y: -1.5
    z: 0.0
  - x: 1.5
    y: -1.5
    z: 0.0
  - x: 1.5
    y: 1.5
    z: 0.0
  - x: -1.5
    y: 1.5
    z: 0.0
"
