#!/usr/bin/env python3
"""
Debug script to test directional mask interpretation.
This will help identify the issue with direction visualization.
"""

def test_mask_interpretation():
    """Test mask bit interpretation"""
    print("=== Mask Bit Interpretation Test ===")
    
    # Test mask = 6 (0b00000110)
    mask = 6
    print(f"\nTesting mask = {mask} (binary: {bin(mask)})")
    
    for bit in range(8):
        is_set = (mask & (1 << bit)) != 0
        direction_names = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
        colors = ["Red", "Blue", "Green", "Blue", "Red", "Blue", "Green", "Blue"]
        vectors = [
            (1.0, 0.0),      # E
            (0.707, 0.707),  # NE  
            (0.0, 1.0),      # N
            (-0.707, 0.707), # NW
            (-1.0, 0.0),     # W
            (-0.707, -0.707),# SW
            (0.0, -1.0),     # S
            (0.707, -0.707)  # SE
        ]
        
        if is_set:
            print(f"  ✓ Bit {bit}: {direction_names[bit]} - {colors[bit]} - Vector({vectors[bit][0]:.3f}, {vectors[bit][1]:.3f})")
        else:
            print(f"    Bit {bit}: {direction_names[bit]} - (not set)")
    
    print(f"\nExpected for mask={mask}:")
    print("  - Blue NE arrow (bit 1): northeast diagonal")
    print("  - Green N arrow (bit 2): north up")
    print("\nActual reported: 'green left and blue diagonal'")
    print("Issue: 'green left' suggests West direction, but bit 4 (W) is NOT set!")

def test_coordinate_systems():
    """Test if there's a coordinate system issue"""
    print("\n=== Coordinate System Analysis ===")
    
    print("ROS/RViz coordinate system:")
    print("  - X: forward (red axis)")
    print("  - Y: left (green axis)") 
    print("  - Z: up (blue axis)")
    
    print("\nMath/Image coordinate system:")
    print("  - X: right")
    print("  - Y: up")
    
    print("\nDirection vector analysis:")
    directions = [
        ("E",  (1.0, 0.0),    "Right"),
        ("NE", (0.707, 0.707), "Up-Right diagonal"),
        ("N",  (0.0, 1.0),    "Up"),
        ("NW", (-0.707, 0.707), "Up-Left diagonal"), 
        ("W",  (-1.0, 0.0),   "Left"),
        ("SW", (-0.707, -0.707), "Down-Left diagonal"),
        ("S",  (0.0, -1.0),   "Down"),
        ("SE", (0.707, -0.707), "Down-Right diagonal")
    ]
    
    for name, vec, desc in directions:
        yaw = __import__('math').atan2(vec[1], vec[0]) * 180 / 3.14159
        print(f"  {name}: ({vec[0]:+.3f}, {vec[1]:+.3f}) -> {desc} (yaw: {yaw:+.1f}°)")

def test_ros_tf_conventions():
    """Test ROS TF frame conventions"""
    print("\n=== ROS TF Frame Convention ===")
    print("Standard ROS frame (REP-103):")
    print("  - X: forward")
    print("  - Y: left") 
    print("  - Z: up")
    
    print("\nArrow directions in RViz should be:")
    print("  - East (1,0): Right on screen")
    print("  - North (0,1): Up on screen")  
    print("  - West (-1,0): Left on screen")
    print("  - South (0,-1): Down on screen")

if __name__ == '__main__':
    test_mask_interpretation()
    test_coordinate_systems()  
    test_ros_tf_conventions()
    
    print("\n=== Debugging Recommendations ===")
    print("1. Check RViz coordinate frame - is 'map' frame oriented correctly?")
    print("2. Verify arrow orientations in RViz match expected directions")
    print("3. Test with simple masks: 1,2,4,8,16,32,64,128 (single bits)")
    print("4. Check if the issue is visualization vs. actual mask logic")
    print("\n✓ Run this in RViz to debug: Add TF display to see frame orientations")
