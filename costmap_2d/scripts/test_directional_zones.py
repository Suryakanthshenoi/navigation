#!/usr/bin/env python

"""
Test script to add directional zones via service calls and demonstrate visualization.
"""

import rospy
from costmap_2d.srv import DirectionalZone, DirectionalZoneRequest
from geometry_msgs.msg import Polygon, Point32
from std_srvs.srv import Empty
import time

def create_zone(name, mask, points):
    """Create a DirectionalZone service request"""
    req = DirectionalZoneRequest()
    req.name = name
    req.mask = mask
    
    polygon = Polygon()
    for x, y in points:
        point = Point32()
        point.x = x
        point.y = y
        point.z = 0.0
        polygon.points.append(point)
    
    req.polygon = polygon
    return req

def main():
    rospy.init_node('test_directional_zones')
    
    # Wait for services
    rospy.loginfo("Waiting for DirectionalLayer services...")
    rospy.wait_for_service('/test_costmap/costmap/directional_layer/add_zone')
    rospy.wait_for_service('/test_costmap/costmap/directional_layer/clear_zones')
    
    add_zone_srv = rospy.ServiceProxy('/test_costmap/costmap/directional_layer/add_zone', DirectionalZone)
    clear_zones_srv = rospy.ServiceProxy('/test_costmap/costmap/directional_layer/clear_zones', Empty)
    
    rospy.loginfo("Adding test zones...")
    
    try:
        # Zone 1: North-South corridor (only N and S allowed)
        zone1 = create_zone("ns_corridor", 68, [(-5, -5), (-4, -5), (-4, 5), (-5, 5)])  # N=bit2, S=bit6: 4+64=68
        response = add_zone_srv(zone1)
        rospy.loginfo("Zone 1 result: %s - %s", response.success, response.message)
        
        time.sleep(1)
        
        # Zone 2: East-West corridor (only E and W allowed) 
        zone2 = create_zone("ew_corridor", 17, [(3, 1), (9, 1), (9, 2), (3, 2)])  # E=bit0, W=bit4: 1+16=17
        response = add_zone_srv(zone2)
        rospy.loginfo("Zone 2 result: %s - %s", response.success, response.message)
        
        time.sleep(1)
        
        # Zone 3: Roundabout (clockwise: only certain directions)
        zone3 = create_zone("roundabout", 51, [(0, 0), (3, 0), (3, 3), (0, 3)])  # E,NE,W: bits 0,1,4: 1+2+16+32=51
        response = add_zone_srv(zone3)
        rospy.loginfo("Zone 3 result: %s - %s", response.success, response.message)
        
        time.sleep(1)
        
        # Zone 4: No-go zone (no directions allowed)
        zone4 = create_zone("blocked_zone", 0, [(6, -4), (8, -4), (8, -2), (6, -2)])
        response = add_zone_srv(zone4)
        rospy.loginfo("Zone 4 result: %s - %s", response.success, response.message)
        
        time.sleep(1)
        
        # Zone 5: Diagonal-only area
        zone5 = create_zone("diagonal_zone", 170, [(-8, 4), (-6, 4), (-6, 6), (-8, 6)])  # Diagonals: bits 1,3,5,7: 2+8+32+128=170
        response = add_zone_srv(zone5)
        rospy.loginfo("Zone 5 result: %s - %s", response.success, response.message)
        
        rospy.loginfo("All zones added successfully!")
        rospy.loginfo("Visualization should now show:")
        rospy.loginfo("  - Red polygons: blocked zones")
        rospy.loginfo("  - Orange/Yellow: restricted zones") 
        rospy.loginfo("  - Green: unrestricted zones")
        rospy.loginfo("  - Colored arrows: allowed movement directions")
        rospy.loginfo("    Red arrows: E/W, Green arrows: N/S, Blue arrows: Diagonals")
        
        # Keep the node running to maintain visualization
        rospy.loginfo("Visualization active. Press Ctrl+C to clear zones and exit.")
        rospy.spin()
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    except KeyboardInterrupt:
        rospy.loginfo("Clearing all zones...")
        try:
            clear_zones_srv()
            rospy.loginfo("Zones cleared successfully!")
        except:
            pass

if __name__ == '__main__':
    main()
