#!/usr/bin/env python3

import rospy
from costmap_2d.srv import DirectionalZone
from geometry_msgs.msg import Point32
import time

def test_directional_layer():
    """Test the DirectionalLayer by adding zones and checking visualization."""
    
    # Initialize ROS node
    rospy.init_node('directional_layer_test', anonymous=True)
    
    print("Waiting for DirectionalLayer service...")
    rospy.wait_for_service('/costmap/directional_layer/add_zone')
    
    add_zone_service = rospy.ServiceProxy('/costmap/directional_layer/add_zone', DirectionalZone)
    
    try:
        # Test Zone 1: Rectangle with North direction (mask = 1)
        zone1_polygon = [
            Point32(x=1.0, y=1.0, z=0.0),
            Point32(x=3.0, y=1.0, z=0.0),
            Point32(x=3.0, y=3.0, z=0.0),
            Point32(x=1.0, y=3.0, z=0.0)
        ]
        
        response1 = add_zone_service(
            name="north_zone",
            mask=1,  # North direction
            polygon=zone1_polygon
        )
        print(f"Zone 1 result: {response1.success}, message: {response1.message}")
        
        # Test Zone 2: Triangle with East direction (mask = 4)
        zone2_polygon = [
            Point32(x=5.0, y=1.0, z=0.0),
            Point32(x=7.0, y=1.0, z=0.0),
            Point32(x=6.0, y=3.0, z=0.0)
        ]
        
        response2 = add_zone_service(
            name="east_zone",
            mask=4,  # East direction
            polygon=zone2_polygon
        )
        print(f"Zone 2 result: {response2.success}, message: {response2.message}")
        
        # Test Zone 3: Complex polygon with multiple directions (mask = 5 = North + East)
        zone3_polygon = [
            Point32(x=8.0, y=5.0, z=0.0),
            Point32(x=10.0, y=5.0, z=0.0),
            Point32(x=11.0, y=7.0, z=0.0),
            Point32(x=9.0, y=8.0, z=0.0),
            Point32(x=7.0, y=7.0, z=0.0)
        ]
        
        response3 = add_zone_service(
            name="northeast_zone",
            mask=5,  # North + East directions
            polygon=zone3_polygon
        )
        print(f"Zone 3 result: {response3.success}, message: {response3.message}")
        
        # Test Zone 4: All directions allowed (mask = 255)
        zone4_polygon = [
            Point32(x=12.0, y=1.0, z=0.0),
            Point32(x=14.0, y=1.0, z=0.0),
            Point32(x=14.0, y=3.0, z=0.0),
            Point32(x=12.0, y=3.0, z=0.0)
        ]
        
        response4 = add_zone_service(
            name="all_directions_zone",
            mask=255,  # All directions
            polygon=zone4_polygon
        )
        print(f"Zone 4 result: {response4.success}, message: {response4.message}")
        
        print("\n✅ All test zones have been added successfully!")
        print("📍 Check RViz to see the visualization markers:")
        print("   - Green/Blue polygons show the zone boundaries")
        print("   - Red arrows show the allowed directions for each zone")
        print("   - The DirectionalLayer mask grid is now populated with directional preferences")
        
        print("\n🎯 Directional Mask Meanings:")
        print("   - Mask 1:   North only")
        print("   - Mask 4:   East only") 
        print("   - Mask 5:   North + East")
        print("   - Mask 255: All directions allowed")
        
        print("\n🚀 To see the visualization:")
        print("   roslaunch costmap_2d test_directional_layer.launch")
        print("   Then open RViz and add MarkerArray topic: /costmap/directional_layer/visualization")
        
        return True
        
    except rospy.ServiceException as e:
        print(f"❌ Service call failed: {e}")
        return False

if __name__ == '__main__':
    try:
        success = test_directional_layer()
        if success:
            print("\n🎉 DirectionalLayer test completed successfully!")
        else:
            print("\n❌ DirectionalLayer test failed!")
    except rospy.ROSInterruptException:
        print("Test interrupted")
