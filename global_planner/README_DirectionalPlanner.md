# DirectionalLayer Integration with Global Planner

This document describes how the global planner has been modified to work with the DirectionalLayer plugin for directional path planning.

## Overview

The global planner now supports directional constraints when the DirectionalLayer plugin is available in the costmap. This allows the planner to consider preferred movement directions when generating paths.

## Integration Features

### ✅ **Implemented Features**

1. **DirectionalLayer Detection**: Automatically detects and uses DirectionalLayer from costmap plugins
2. **Dijkstra Algorithm Support**: Modified to respect directional constraints during expansion  
3. **A* Algorithm Support**: Enhanced with directional penalties and constraint checking
4. **Configurable Penalties**: Adjustable penalty factors for disallowed directions
5. **Backward Compatibility**: Works normally when DirectionalLayer is not present

### 🔧 **Modified Components**

#### GlobalPlanner Class (`planner_core.h/cpp`)
- Added DirectionalLayer detection and integration
- New parameters: `use_directional_constraints`, `directional_penalty_factor`
- Automatic plugin discovery from layered costmap

#### Expander Base Class (`expander.h/cpp`)
- Added directional constraint checking methods
- Directional penalty calculation
- Movement direction validation

#### DijkstraExpansion (`dijkstra.cpp`)
- Modified neighbor expansion to check directional constraints
- Applied directional penalties to edge costs
- Skips disallowed directions during search

#### AStarExpansion (`astar.cpp`)
- Enhanced neighbor addition with directional checks
- Cost adjustment based on directional penalties
- Early termination for blocked directions

## Configuration

### Parameters

Add these parameters to your global planner configuration:

```yaml
global_planner:
  # Enable directional constraints
  use_directional_constraints: true
  
  # Penalty factor for disallowed directions
  # 1.0 = no penalty, higher values = stronger penalties
  directional_penalty_factor: 3.0
```

### Full Configuration Example

```yaml
move_base:
  # Global Costmap with DirectionalLayer
  global_costmap:
    plugins:
      - {name: static_layer, type: "costmap_2d::StaticLayer"}
      - {name: directional_layer, type: "costmap_2d::DirectionalLayer"}
    
    directional_layer:
      enabled: true
      default_mask: 255
      zones_file: ""
      visualization_topic: "directional_visualization"
  
  # Global Planner with DirectionalLayer support
  base_global_planner: "global_planner/GlobalPlanner"
  
  GlobalPlanner:
    use_directional_constraints: true
    directional_penalty_factor: 2.5
    use_dijkstra: true
    allow_unknown: false
```

## How It Works

Direction	Bit	Mask        Value	        Binary	Arrow Color	Description
East	    0	1	        0b00000001	    🔴 Red	Point right (→)
Northeast	1	2	        0b00000010	    🔵 Blue	Point diagonal up-right (↗)
North	    2	4	        0b00000100	    🟢 Green	Point up (↑)
Northwest	3	8	        0b00001000	    🔵 Blue	Point diagonal up-left (↖)
West	    4	16	        0b00010000	    🔴 Red	Point left (←)
Southwest	5	32	        0b00100000	    🔵 Blue	Point diagonal down-left (↙)
South	    6	64	        0b01000000	    🟢 Green	Point down (↓)
Southeast	7	128	        0b10000000	    🔵 Blue	Point diagonal down-right (↘)

### 1. **Initialization**
```cpp
// Global planner detects DirectionalLayer during initialization
directional_layer_ = getDirectionalLayer();
if (directional_layer_ && use_directional_constraints_) {
    planner_->setDirectionalLayer(directional_layer_);
}
```

### 2. **Path Search with Directional Constraints**

**Dijkstra Expansion:**
```cpp
// Check direction before expanding to neighbor
if (directional_layer_ == NULL || isDirectionAllowed(current_x, current_y, neighbor_x, neighbor_y)) {
    // Apply directional penalty to cost
    float penalty = getDirectionalPenalty(current_x, current_y, neighbor_x, neighbor_y);
    float adjusted_cost = base_cost * penalty;
    
    // Add neighbor to search queue
    push_next(neighbor);
}
```

**A* Expansion:**
```cpp
// Skip neighbor if direction not allowed
if (directional_layer_ && !isDirectionAllowed(from_x, from_y, to_x, to_y)) {
    return;  // Skip this neighbor
}

// Apply penalty to cost calculation
float directional_penalty = getDirectionalPenalty(from_x, from_y, to_x, to_y);
float adjusted_cost = (costs[next_i] + neutral_cost_) * directional_penalty;
```

### 3. **Direction Mapping**

8-connected movement directions are mapped to DirectionalLayer bits:

```cpp
// Movement direction to bit mapping
if (dx == 1 && dy == 0)       direction_bit = 0;  // East
else if (dx == 1 && dy == 1)  direction_bit = 1;  // Northeast  
else if (dx == 0 && dy == 1)  direction_bit = 2;  // North
else if (dx == -1 && dy == 1) direction_bit = 3;  // Northwest
else if (dx == -1 && dy == 0) direction_bit = 4;  // West
else if (dx == -1 && dy == -1) direction_bit = 5; // Southwest
else if (dx == 0 && dy == -1) direction_bit = 6;  // South
else if (dx == 1 && dy == -1) direction_bit = 7;  // Southeast
```

## Testing

### 1. **Basic Functionality Test**
```bash
# Start roscore
roscore

# Launch costmap with DirectionalLayer
roslaunch costmap_2d test_directional_layer.launch

# Add directional zones
rosservice call /move_base/global_costmap/directional_layer/add_zone "..."

# Request path planning
rosservice call /move_base/GlobalPlanner/make_plan "..."
```

### 2. **Performance Comparison**
```bash
# Test without directional constraints
rosparam set /move_base/GlobalPlanner/use_directional_constraints false

# Test with directional constraints  
rosparam set /move_base/GlobalPlanner/use_directional_constraints true
rosparam set /move_base/GlobalPlanner/directional_penalty_factor 2.0
```

## Algorithm Behavior

### Without DirectionalLayer
- **Standard Operation**: Normal Dijkstra/A* pathfinding
- **All Directions**: 8-connected neighbor expansion
- **Equal Costs**: Uniform edge costs based on costmap only

### With DirectionalLayer
- **Directional Filtering**: Disallowed directions are skipped entirely
- **Cost Penalties**: Allowed but discouraged directions get penalty multipliers  
- **Zone-Aware Planning**: Different areas have different directional preferences
- **Visualization Support**: Plans show respect for directional constraints

## Performance Impact

### Computational Overhead
- **Minimal**: Direction checking is O(1) per neighbor expansion
- **Memory**: No additional memory overhead
- **Cache Friendly**: DirectionalLayer mask access is efficient

### Planning Quality
- **Improved Compliance**: Paths follow intended traffic patterns
- **Contextual Routing**: Different behaviors in different zones
- **Predictable Behavior**: Consistent directional preferences

## Use Cases

### 1. **Warehouse Navigation**
```yaml
# Loading dock - vehicles must approach from south
- name: "loading_dock"
  mask: 1  # North only (bit 2)
  polygon: [[10, 5], [12, 5], [12, 7], [10, 7]]
  
# Corridor - bidirectional but preferred east-west
- name: "main_corridor"  
  mask: 17  # East + West (bits 0,4: 1 + 16)
  polygon: [[0, 10], [20, 10], [20, 12], [0, 12]]
```

### 2. **Road-like Environments**
```yaml
# One-way street going east
- name: "eastbound_street"
  mask: 4  # East only (bit 2)
  
# Roundabout - clockwise only
- name: "roundabout"
  mask: 170  # NE,SE,SW,NW (bits 1,3,5,7: 2+8+32+128)
```

### 3. **Multi-Robot Coordination**
```yaml
# Conflict zones with directional flow
- name: "intersection_north"
  mask: 1   # North priority
- name: "intersection_east"  
  mask: 4   # East priority
```

## Troubleshooting

### Common Issues

**1. DirectionalLayer Not Found**
```
GlobalPlanner: DirectionalLayer not found but directional constraints requested
```
**Solution**: Ensure DirectionalLayer is properly configured in costmap plugins

**2. No Directional Effect**
```
# Check if constraints are enabled
rosparam get /move_base/GlobalPlanner/use_directional_constraints

# Check penalty factor
rosparam get /move_base/GlobalPlanner/directional_penalty_factor
```

**3. Paths Ignore Directional Zones**
- Verify DirectionalLayer service is working: `rosservice list | grep directional`
- Check zone masks with: `rosservice call /costmap/directional_layer/add_zone`
- Enable visualization to see zones and constraints

### Performance Optimization

```yaml
# For better performance in complex environments
GlobalPlanner:
  directional_penalty_factor: 2.0  # Lower penalty = faster search
  use_dijkstra: true              # Generally faster than A*
  allow_unknown: false            # Stricter collision checking
```

## Future Enhancements

### Planned Features
- [ ] **Dynamic Directional Updates**: Real-time constraint modification
- [ ] **Multi-level Penalties**: Graduated penalty system instead of binary allow/disallow
- [ ] **Temporal Constraints**: Time-based directional preferences
- [ ] **Vehicle-specific Directions**: Different constraints per robot type

### Integration Opportunities  
- [ ] **Local Planner Integration**: Pass directional preferences to trajectory planning
- [ ] **Multi-robot Coordination**: Shared directional constraints
- [ ] **Learning Components**: Adaptive directional preferences based on experience

## Contributing

When modifying the directional global planner:

1. **Maintain Compatibility**: Ensure backward compatibility with non-directional setups
2. **Test Both Modes**: Verify functionality with and without DirectionalLayer
3. **Document Changes**: Update configuration examples and parameter descriptions
4. **Performance Testing**: Benchmark planning time impact

## References

- [DirectionalLayer Plugin Documentation](README_DirectionalLayer.md)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [Global Planner Package](http://wiki.ros.org/global_planner)
- [Costmap 2D Package](http://wiki.ros.org/costmap_2d)
