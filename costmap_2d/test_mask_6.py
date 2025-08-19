#!/usr/bin/env python3
"""
Test script to debug directional mask = 6 issue.
"""

import rospy
from costmap_2d.srv import DirectionalZone
from geometry_msgs.msg import Point32

def test_mask_6():
    """Test mask = 6 specifically"""
    rospy.init_node('test_mask_6', anonymous=True)
    
    print("=== Testing Mask = 6 ===")
    print("Binary: 0b00000110")
    print("Expected arrows:")
    print("  - Bit 1 (NE): Blue northeast diagonal")
    print("  - Bit 2 (N):  Green north up")
    print("Should NOT see any other arrows!")
    print()
    
    # Wait for service
    print("Waiting for DirectionalLayer service...")
    try:
        rospy.wait_for_service('/costmap/directional_layer/add_zone', timeout=5.0)
    except rospy.ROSException:
        print("❌ Service not available. Make sure DirectionalLayer is running.")
        return False
    
    add_zone_service = rospy.ServiceProxy('/costmap/directional_layer/add_zone', DirectionalZone)
    
    # Create a simple square zone
    zone_polygon = [
        Point32(x=2.0, y=2.0, z=0.0),
        Point32(x=4.0, y=2.0, z=0.0),
        Point32(x=4.0, y=4.0, z=0.0),
        Point32(x=2.0, y=4.0, z=0.0)
    ]
    
    try:
        response = add_zone_service(
            name="test_mask_6",
            mask=6,  # 0b00000110 = bits 1 and 2 set
            polygon=zone_polygon
        )
        
        if response.success:
            print(f"✅ Zone added successfully: {response.message}")
            print("\nNow check RViz visualization:")
            print("1. You should see exactly 2 arrows:")
            print("   - Blue arrow pointing northeast (↗)")
            print("   - Green arrow pointing north (↑)")
            print("2. If you see a 'green left' arrow, that's the bug!")
            print("3. Check the terminal output for debug messages")
            return True
        else:
            print(f"❌ Failed to add zone: {response.message}")
            return False
            
    except rospy.ServiceException as e:
        print(f"❌ Service call failed: {e}")
        return False

if __name__ == '__main__':
    try:
        success = test_mask_6()
        if success:
            print("\n🔍 Look for debug output in the DirectionalLayer node terminal")
            print("   The debug messages should show:")
            print("   - Zone 'test_mask_6', dir=1, mask=6, vec=(0.707,0.707), yaw=45.0°")
            print("   - Zone 'test_mask_6', dir=2, mask=6, vec=(0.000,1.000), yaw=90.0°")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Test interrupted")
