# Global Planner with Strict Directional Constraints

This enhanced global planner integrates with the DirectionalLayer to enforce strict directional constraints during path planning.

## 🎯 Features

### Normal Mode (`oneway_strict_mode: false`)
- Applies **cost penalties** to disallowed directions
- Robot can still move through restricted areas but with higher cost
- Useful for preferences rather than hard constraints

### Strict Mode (`oneway_strict_mode: true`) 
- Treats disallowed directions as **lethal obstacles**
- Completely blocks movement in forbidden directions
- Perfect for one-way roads, corridors, traffic rules

## ⚙️ Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_directional_constraints` | bool | false | Enable DirectionalLayer integration |
| `oneway_strict_mode` | bool | false | **STRICT: Treat disallowed directions as lethal** |
| `directional_penalty_factor` | double | 2.0 | Cost multiplier for disallowed directions (normal mode only) |

## 🚀 Usage Examples

### Basic Setup
```yaml
global_planner:
  use_directional_constraints: true
  oneway_strict_mode: true  # Enable strict enforcement
```

### Launch File Integration
```xml
<node name="move_base" pkg="move_base" type="move_base">
  <rosparam file="$(find global_planner)/config/strict_directional_planner.yaml" command="load"/>
  <param name="base_global_planner" value="global_planner/GlobalPlanner"/>
</node>
```

## 🔧 How It Works

### Algorithm Integration

The directional constraints are enforced at the **neighbor expansion level** in both Dijkstra and A* algorithms:

#### Normal Mode
```cpp
// Apply penalty to disallowed directions
cost *= getDirectionalPenalty(from_x, from_y, to_x, to_y);
```

#### Strict Mode  
```cpp
// Skip neighbors that are directionally forbidden
if (!isDirectionLethal(from_x, from_y, to_x, to_y)) {
    // Add neighbor to expansion queue
    addNeighbor(neighbor);
}
```

### Direction Mapping

The planner checks each of the 8 possible movement directions:

| Direction | Bit | Mask | Check |
|-----------|-----|------|-------|
| East → | 0 | `1` | `(mask & 1) != 0` |
| Northeast ↗ | 1 | `2` | `(mask & 2) != 0` |  
| North ↑ | 2 | `4` | `(mask & 4) != 0` |
| Northwest ↖ | 3 | `8` | `(mask & 8) != 0` |
| West ← | 4 | `16` | `(mask & 16) != 0` |
| Southwest ↙ | 5 | `32` | `(mask & 32) != 0` |
| South ↓ | 6 | `64` | `(mask & 64) != 0` |
| Southeast ↘ | 7 | `128` | `(mask & 128) != 0` |

## 🎯 Use Cases

### One-Way Roads
```bash
# Allow only eastbound traffic
rosservice call /move_base/global_costmap/directional_layer/add_zone "
name: 'highway_eastbound'
mask: 1
polygon: {...}
"
```

### Loading Dock Approach
```bash  
# Vehicles must approach from south (go north)
rosservice call /move_base/global_costmap/directional_layer/add_zone "
name: 'loading_dock'
mask: 4
polygon: {...}
"
```

### Narrow Corridor
```bash
# Allow forward/backward movement only  
rosservice call /move_base/global_costmap/directional_layer/add_zone "
name: 'narrow_corridor'
mask: 68  # North + South (4 + 64)
polygon: {...}
"
```

## 🧪 Testing Strict Mode

### Test Setup
1. **Enable strict mode** in global planner config
2. **Add directional zones** with restricted masks
3. **Plan paths** that would violate directional constraints
4. **Verify** that paths avoid forbidden directions completely

### Example Test
```bash
# Clear existing zones
rosservice call /move_base/global_costmap/directional_layer/clear_zones "{}"

# Add east-only zone
rosservice call /move_base/global_costmap/directional_layer/add_zone "
name: 'test_east_only'  
mask: 1
polygon: {points: [{x: 0, y: 0}, {x: 2, y: 0}, {x: 2, y: 2}, {x: 0, y: 2}]}
"

# Plan a path through the zone - should only use eastbound movements
```

## ⚠️ Important Notes

### Performance Impact
- **Strict mode** reduces the search space by eliminating forbidden directions
- This can actually **improve** planning performance in constrained environments
- **Normal mode** may increase computational cost due to penalty calculations

### Path Quality
- **Strict mode** ensures **100% compliance** with directional constraints
- May result in longer paths when direct routes are forbidden
- Guarantees feasible paths respect traffic rules

### Fallback Behavior
- If **no valid path** exists due to strict constraints, planner will fail
- Consider providing **escape zones** with relaxed constraints
- Use **multiple mask values** to create alternative routes

## 🔍 Debugging

### Visualization
- DirectionalLayer publishes **MarkerArray** showing zones and allowed directions
- **Green arrows**: Allowed directions in current zone
- **Zone colors**: Indicate constraint severity

### Logging  
```bash
# Enable debug output
rosrun rqt_logger_level rqt_logger_level
# Set global_planner to DEBUG level
```

### Common Issues
1. **No path found**: Constraints too restrictive
2. **Ignoring constraints**: `use_directional_constraints: false`
3. **Wrong directions**: Check DirectionalLayer mask values

## 📋 Configuration Template

```yaml
# Complete configuration for strict directional planning
move_base:
  base_global_planner: "global_planner/GlobalPlanner"
  
  GlobalPlanner:
    use_directional_constraints: true
    oneway_strict_mode: true          # ENABLE STRICT MODE
    directional_penalty_factor: 2.0   # Unused in strict mode
    allow_unknown: true
    use_dijkstra: true
    
global_costmap:
  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}  
    - {name: directional_layer, type: "costmap_2d::DirectionalLayer"}
    
  directional_layer:
    enabled: true
    default_mask: 255  # All directions allowed by default
```

This configuration ensures that the global planner will **strictly enforce** directional constraints, treating forbidden movement directions as impassable obstacles.
