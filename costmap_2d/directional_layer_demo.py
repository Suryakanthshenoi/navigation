#!/usr/bin/env python3
"""
DirectionalLayer Demo Script

This script demonstrates the DirectionalLayer functionality:
1. Shows service message structure
2. Explains directional mask bit encoding  
3. Provides usage examples

The DirectionalLayer stores 8-bit directional preferences per grid cell:
- Bit 0: North (↑)
- Bit 1: Northeast (↗)
- Bit 2: East (→)
- Bit 3: Southeast (↘)
- Bit 4: South (↓)
- Bit 5: Southwest (↙)
- Bit 6: West (←)
- Bit 7: Northwest (↖)
"""

import rospy
from costmap_2d.srv import DirectionalZone, DirectionalZoneRequest, DirectionalZoneResponse
from geometry_msgs.msg import Point32

def explain_directional_masks():
    """Explain the directional mask bit encoding."""
    print("🧭 DirectionalLayer Mask Encoding:")
    print("   Each grid cell stores an 8-bit mask representing allowed directions:")
    print("   Bit 0 (1):   North     ↑")
    print("   Bit 1 (2):   Northeast ↗")  
    print("   Bit 2 (4):   East      →")
    print("   Bit 3 (8):   Southeast ↘")
    print("   Bit 4 (16):  South     ↓")
    print("   Bit 5 (32):  Southwest ↙")
    print("   Bit 6 (64):  West      ←")
    print("   Bit 7 (128): Northwest ↖")
    print()
    
    print("📋 Common Mask Values:")
    print("   255 = All directions allowed (default)")
    print("   1   = North only")
    print("   4   = East only") 
    print("   5   = North + East (1 + 4)")
    print("   15  = North/Northeast/East/Southeast (1+2+4+8)")
    print("   0   = No movement allowed")
    print()

def show_service_structure():
    """Show the DirectionalZone service message structure."""
    print("🔌 DirectionalZone Service Structure:")
    
    # Request
    req = DirectionalZoneRequest()
    print(f"   Request fields: {req.__slots__}")
    print("   - name: string identifier for the zone")
    print("   - mask: 8-bit directional preference mask")
    print("   - polygon: list of Point32 defining zone boundary")
    print()
    
    # Response  
    resp = DirectionalZoneResponse()
    print(f"   Response fields: {resp.__slots__}")
    print("   - success: boolean indicating if zone was added")
    print("   - message: string with result details")
    print()

def create_sample_zones():
    """Create sample directional zones for demonstration."""
    print("🎯 Sample Directional Zones:")
    
    zones = [
        {
            "name": "loading_dock", 
            "mask": 1,  # North only
            "description": "Loading dock - vehicles must approach from south (go north)",
            "polygon": [
                Point32(x=10.0, y=5.0, z=0.0),
                Point32(x=12.0, y=5.0, z=0.0), 
                Point32(x=12.0, y=7.0, z=0.0),
                Point32(x=10.0, y=7.0, z=0.0)
            ]
        },
        {
            "name": "corridor",
            "mask": 5,  # North + East (1 + 4)
            "description": "Narrow corridor - only north/east movement allowed",
            "polygon": [
                Point32(x=15.0, y=10.0, z=0.0),
                Point32(x=20.0, y=10.0, z=0.0),
                Point32(x=20.0, y=12.0, z=0.0), 
                Point32(x=15.0, y=12.0, z=0.0)
            ]
        },
        {
            "name": "roundabout",
            "mask": 10,  # Northeast + Southeast (2 + 8) 
            "description": "Roundabout - only clockwise movement",
            "polygon": [
                Point32(x=5.0, y=15.0, z=0.0),
                Point32(x=8.0, y=15.0, z=0.0),
                Point32(x=8.0, y=18.0, z=0.0),
                Point32(x=5.0, y=18.0, z=0.0)
            ]
        }
    ]
    
    for zone in zones:
        print(f"   Zone: {zone['name']}")
        print(f"   Mask: {zone['mask']} ({bin(zone['mask'])})")
        print(f"   Description: {zone['description']}")
        print(f"   Polygon: {len(zone['polygon'])} points")
        print()

def show_visualization_info():
    """Show information about visualization features."""
    print("📺 Visualization Features:")
    print("   The DirectionalLayer publishes MarkerArray messages for RViz:")
    print("   - Polygon outlines show zone boundaries (colored by mask)")
    print("   - Arrow markers show allowed directions at zone centers")
    print("   - Topic: /costmap/directional_layer/visualization")
    print()
    
    print("🎨 Visualization Colors:")
    print("   - Blue/Green polygons: zone boundaries") 
    print("   - Red arrows: allowed movement directions")
    print("   - Arrow size indicates direction strength")
    print()

def main():
    """Main demonstration function."""
    print("=" * 60)
    print("🗺️  DirectionalLayer Plugin Demonstration")
    print("=" * 60)
    print()
    
    # Check if running in ROS environment
    try:
        rospy.init_node('directional_layer_demo', anonymous=True)
        ros_available = True
    except:
        ros_available = False
        print("⚠️  ROS not available - showing offline demonstration")
        print()
    
    # Show explanations
    explain_directional_masks()
    show_service_structure() 
    create_sample_zones()
    show_visualization_info()
    
    print("🚀 To use the DirectionalLayer:")
    print("   1. Add 'directional_layer' to your costmap plugins")
    print("   2. Configure parameters in costmap YAML")
    print("   3. Use the add_zone service to create directional zones")
    print("   4. View visualization in RViz with MarkerArray")
    print()
    
    if ros_available:
        print("✅ ROS is available - you can test the actual service!")
        print("   Run: roslaunch costmap_2d test_directional_layer.launch")
    else:
        print("ℹ️  Start ROS to test the actual DirectionalLayer service")
    
    print()
    print("🎉 DirectionalLayer demonstration complete!")

if __name__ == '__main__':
    main()
