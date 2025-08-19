# DirectionalLayer Visualization

The DirectionalLayer plugin now includes comprehensive visualization capabilities that publish the directional zones as marker arrays with arrows showing allowed movement directions.

## Visualization Features

### Zone Visualization
- **Polygon Outlines**: Each directional zone is visualized as a colored polygon outline
- **Color Coding**: 
  - 🟢 **Green**: All directions allowed (mask = 0xFF)
  - 🟡 **Yellow**: Moderately restricted (4-7 directions allowed)
  - 🟠 **Orange**: Highly restricted (1-3 directions allowed)  
  - 🔴 **Red**: Blocked zone (no directions allowed, mask = 0x00)

### Direction Arrows
- **Arrow Markers**: Arrows at the center of each zone show allowed movement directions
- **Color Coding**:
  - 🔴 **Red Arrows**: East/West directions (horizontal movement)
  - 🟢 **Green Arrows**: North/South directions (vertical movement)
  - 🔵 **Blue Arrows**: Diagonal directions (NE, NW, SE, SW)

### Direction Bit Encoding
The plugin uses 8-bit masks to represent allowed directions in 8-connected movement:
```
bit 0: E  (dx=+1, dy= 0) - East
bit 1: NE (dx=+1, dy=+1) - Northeast  
bit 2: N  (dx= 0, dy=+1) - North
bit 3: NW (dx=-1, dy=+1) - Northwest
bit 4: W  (dx=-1, dy= 0) - West
bit 5: SW (dx=-1, dy=-1) - Southwest
bit 6: S  (dx= 0, dy=-1) - South
bit 7: SE (dx=+1, dy=-1) - Southeast
```

## Configuration Parameters

Add these parameters to your costmap configuration to control visualization:

```yaml
directional_layer:
  # Visualization settings
  enable_visualization: true      # Enable marker publishing
  visualization_rate: 2.0         # Update rate (Hz)
  visualization_frame: "map"      # Frame for markers
  arrow_scale: 1.0               # Size of direction arrows
```

## ROS Topics

### Published Topics
- `directional_zones` (visualization_msgs/MarkerArray): Visualization markers for zones and arrows

## Testing the Visualization

1. **Launch the test setup**:
   ```bash
   roslaunch costmap_2d test_directional_layer.launch
   ```

2. **View in RViz**: The launch file will open RViz with the appropriate configuration

3. **Add zones dynamically**: The test script automatically adds several example zones:
   - North-South corridor (only N/S movement)
   - East-West corridor (only E/W movement) 
   - Roundabout (selective directional flow)
   - Blocked zone (no movement)
   - Diagonal-only zone

## Service Interface

### Add Zone Service
```bash
rosservice call /costmap/directional_layer/add_zone "name: 'test_zone'
mask: 17
polygon:
  points:
  - {x: 1.0, y: 1.0, z: 0.0}  
  - {x: 3.0, y: 1.0, z: 0.0}
  - {x: 3.0, y: 3.0, z: 0.0}
  - {x: 1.0, y: 3.0, z: 0.0}"
```

### Clear Zones Service  
```bash
rosservice call /costmap/directional_layer/clear_zones
```

## Example Mask Values

- `255` (0xFF): All directions allowed
- `17` (0x11): Only East and West (bits 0 and 4)
- `68` (0x44): Only North and South (bits 2 and 6)
- `170` (0xAA): Only diagonals (bits 1,3,5,7)
- `0` (0x00): No movement allowed (blocked)

The visualization updates automatically when zones are added, modified, or cleared through service calls.
