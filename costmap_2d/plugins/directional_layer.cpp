/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Your Name
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Your Name
 *********************************************************************/

#include <costmap_2d/directional_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpcException.h>
#include <boost/bind.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PLUGINLIB_EXPORT_CLASS(costmap_2d::DirectionalLayer, costmap_2d::Layer)

namespace costmap_2d
{

DirectionalLayer::DirectionalLayer() 
  : size_x_(0)
  , size_y_(0) 
  , resolution_(0.0)
  , origin_x_(0.0)
  , origin_y_(0.0)
  , zones_file_("")
  , auto_load_zones_(true)
  , default_mask_(0xFF)  // All directions allowed by default
  , enable_visualization_(true)
  , visualization_rate_(1.0)
  , visualization_frame_("map")
  , arrow_scale_(0.5)
{
}

DirectionalLayer::~DirectionalLayer()
{
  if (add_zone_srv_)
    add_zone_srv_.shutdown();
  if (clear_zones_srv_)
    clear_zones_srv_.shutdown();
  if (visualization_timer_)
    visualization_timer_.stop();
}

void DirectionalLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // Read parameters
  nh.param("zones_file", zones_file_, std::string(""));
  nh.param("auto_load_zones", auto_load_zones_, true);
  int default_mask_int;
  nh.param("default_mask", default_mask_int, 255);  // 0xFF
  default_mask_ = static_cast<uint8_t>(default_mask_int);
  nh.param("enable_visualization", enable_visualization_, true);
  nh.param("visualization_rate", visualization_rate_, 1.0);
  nh.param("visualization_frame", visualization_frame_, std::string("map"));
  nh.param("arrow_scale", arrow_scale_, 0.5);

  // Initialize mask grid  
  matchSize();

  // Set up services
  add_zone_srv_ = nh.advertiseService("add_zone", &DirectionalLayer::addZoneCallback, this);
  clear_zones_srv_ = nh.advertiseService("clear_zones", &DirectionalLayer::clearZonesCallback, this);

  // Set up visualization
  if (enable_visualization_)
  {
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("directional_zones", 1, true);
  }

  // Load zones from file if specified
  if (auto_load_zones_ && !zones_file_.empty())
  {
    loadZonesFromParams();
  }

  ROS_INFO("DirectionalLayer initialized with default mask 0x%02X", default_mask_);
}

void DirectionalLayer::matchSize()
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  
  if (!layered_costmap_)
    return;

  Costmap2D* costmap = layered_costmap_->getCostmap();
  
  size_x_ = costmap->getSizeInCellsX();
  size_y_ = costmap->getSizeInCellsY();
  resolution_ = costmap->getResolution();
  origin_x_ = costmap->getOriginX();
  origin_y_ = costmap->getOriginY();

  initializeMaskGrid();
}

void DirectionalLayer::initializeMaskGrid()
{
  unsigned int grid_size = size_x_ * size_y_;
  mask_grid_.assign(grid_size, default_mask_);
  
  ROS_DEBUG("DirectionalLayer: initialized mask grid %dx%d = %d cells", 
           size_x_, size_y_, grid_size);
}

void DirectionalLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  // This layer doesn't need to update bounds since it doesn't modify the master costmap
  // The mask grid is queried directly by planners
}

void DirectionalLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                  int min_i, int min_j, int max_i, int max_j)
{
  // DirectionalLayer does NOT modify costs in the master grid
  // It only provides directional metadata through getMask()
}

void DirectionalLayer::reset()
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  initializeMaskGrid();
  
  // Re-apply all zones
  for (const auto& zone : zones_)
  {
    applyZone(zone);
  }
}

uint8_t DirectionalLayer::getMask(unsigned int mx, unsigned int my) const
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  
  if (mx >= size_x_ || my >= size_y_)
  {
    ROS_WARN_THROTTLE(1.0, "DirectionalLayer: getMask called with out-of-bounds coordinates (%d,%d)", mx, my);
    return 0x00;  // No movement allowed for out-of-bounds
  }
  
  unsigned int index = my * size_x_ + mx;
  return mask_grid_[index];
}

bool DirectionalLayer::isDirectionAllowed(unsigned int mx, unsigned int my, uint8_t direction) const
{
  if (direction > 7)
  {
    ROS_WARN("DirectionalLayer: invalid direction %d (must be 0-7)", direction);
    return false;
  }
  
  uint8_t mask = getMask(mx, my);
  return (mask & (1 << direction)) != 0;
}

uint8_t DirectionalLayer::getDirectionFromDelta(int dx, int dy)
{
  // Map 8-connected neighbor deltas to direction bits
  // bit 0: E  (dx=+1, dy= 0)
  // bit 1: NE (dx=+1, dy=+1) 
  // bit 2: N  (dx= 0, dy=+1)
  // bit 3: NW (dx=-1, dy=+1)
  // bit 4: W  (dx=-1, dy= 0)
  // bit 5: SW (dx=-1, dy=-1)
  // bit 6: S  (dx= 0, dy=-1)
  // bit 7: SE (dx=+1, dy=-1)

  if (dx == 1 && dy == 0)  return 0;  // E
  if (dx == 1 && dy == 1)  return 1;  // NE
  if (dx == 0 && dy == 1)  return 2;  // N
  if (dx == -1 && dy == 1) return 3;  // NW
  if (dx == -1 && dy == 0) return 4;  // W
  if (dx == -1 && dy == -1) return 5; // SW
  if (dx == 0 && dy == -1) return 6;  // S
  if (dx == 1 && dy == -1) return 7;  // SE
  
  return 255;  // Invalid direction
}

bool DirectionalLayer::addZoneCallback(costmap_2d::DirectionalZone::Request& req,
                                      costmap_2d::DirectionalZone::Response& res)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  
  try
  {
    DirectionalZoneData zone;
    zone.name = req.name;
    zone.mask = req.mask;
    
    // Convert polygon
    for (const auto& point : req.polygon.points)
    {
      zone.polygon.push_back(point);
    }
    
    if (zone.polygon.size() < 3)
    {
      res.success = false;
      res.message = "Polygon must have at least 3 vertices";
      return true;
    }
    
    // Add zone and apply to grid
    zones_.push_back(zone);
    applyZone(zone);
    
    res.success = true;
    res.message = "Zone added successfully";
    
    ROS_INFO("DirectionalLayer: Added zone '%s' with mask 0x%02X (%d) and %zu vertices", 
             zone.name.c_str(), zone.mask, zone.mask, zone.polygon.size());
    
    // Debug: Show which directions are allowed
    std::string allowed_dirs = "";
    const char* dir_names[] = {"E", "NE", "N", "NW", "W", "SW", "S", "SE"};
    for (int i = 0; i < 8; i++) {
        if (zone.mask & (1 << i)) {
            if (!allowed_dirs.empty()) allowed_dirs += ", ";
            allowed_dirs += dir_names[i];
        }
    }
    ROS_INFO("DirectionalLayer: Zone '%s' allows directions: [%s]", zone.name.c_str(), allowed_dirs.c_str());
    
    // Trigger immediate visualization update
    if (enable_visualization_)
    {
      publishVisualization();
    }
    
    return true;
  }
  catch (const std::exception& e)
  {
    res.success = false;
    res.message = std::string("Error adding zone: ") + e.what();
    ROS_ERROR("DirectionalLayer: %s", res.message.c_str());
    return true;
  }
}

bool DirectionalLayer::clearZonesCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  
  zones_.clear();
  reset();  // Reset grid and re-apply (now empty) zones
  
  // Trigger immediate visualization update
  if (enable_visualization_)
  {
    publishVisualization();
  }
  
  ROS_INFO("DirectionalLayer: Cleared all zones");
  return true;
}

void DirectionalLayer::loadZonesFromParams()
{
  ros::NodeHandle nh("~/" + name_);
  
  try
  {
    XmlRpc::XmlRpcValue zones_config;
    if (!nh.getParam("zones", zones_config))
    {
      ROS_WARN("DirectionalLayer: No 'zones' parameter found");
      return;
    }
    
    if (zones_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("DirectionalLayer: 'zones' parameter must be an array");
      return;
    }
    
    for (int i = 0; i < zones_config.size(); ++i)
    {
      XmlRpc::XmlRpcValue& zone_config = zones_config[i];
      
      if (zone_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("DirectionalLayer: Zone %d must be a struct", i);
        continue;
      }
      
      DirectionalZoneData zone;
      
      // Parse zone name
      if (zone_config.hasMember("name"))
        zone.name = static_cast<std::string>(zone_config["name"]);
      else
        zone.name = "zone_" + std::to_string(i);
      
      // Parse mask
      if (!zone_config.hasMember("mask"))
      {
        ROS_ERROR("DirectionalLayer: Zone '%s' missing 'mask' field", zone.name.c_str());
        continue;
      }
      zone.mask = static_cast<int>(zone_config["mask"]);
      
      // Parse polygon
      if (!zone_config.hasMember("polygon"))
      {
        ROS_ERROR("DirectionalLayer: Zone '%s' missing 'polygon' field", zone.name.c_str());
        continue;
      }
      
      XmlRpc::XmlRpcValue& polygon_config = zone_config["polygon"];
      if (polygon_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("DirectionalLayer: Zone '%s' polygon must be an array", zone.name.c_str());
        continue;
      }
      
      for (int j = 0; j < polygon_config.size(); ++j)
      {
        XmlRpc::XmlRpcValue& point_config = polygon_config[j];
        if (point_config.getType() != XmlRpc::XmlRpcValue::TypeArray || point_config.size() != 2)
        {
          ROS_ERROR("DirectionalLayer: Zone '%s' polygon point %d must be [x, y]", zone.name.c_str(), j);
          continue;
        }
        
        geometry_msgs::Point32 point;
        point.x = static_cast<double>(point_config[0]);
        point.y = static_cast<double>(point_config[1]);
        point.z = 0.0;
        zone.polygon.push_back(point);
      }
      
      if (zone.polygon.size() >= 3)
      {
        zones_.push_back(zone);
        applyZone(zone);
        ROS_INFO("DirectionalLayer: Loaded zone '%s' with mask 0x%02X", 
                zone.name.c_str(), zone.mask);
      }
      else
      {
        ROS_ERROR("DirectionalLayer: Zone '%s' has insufficient vertices (%zu)", 
                 zone.name.c_str(), zone.polygon.size());
      }
    }
  }
  catch (const XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("DirectionalLayer: Error parsing zones parameter: %s", e.getMessage().c_str());
  }
}

void DirectionalLayer::applyZone(const DirectionalZoneData& zone)
{
  if (mask_grid_.empty())
  {
    ROS_WARN("DirectionalLayer: Cannot apply zone - mask grid not initialized");
    return;
  }
  
  // Apply zone mask to all cells inside the polygon
  for (unsigned int mx = 0; mx < size_x_; ++mx)
  {
    for (unsigned int my = 0; my < size_y_; ++my)
    {
      // Convert grid coordinates to world coordinates
      double wx, wy;
      maskToWorld(mx, my, wx, wy);
      
      geometry_msgs::Point32 point;
      point.x = wx;
      point.y = wy;
      point.z = 0.0;
      
      if (pointInPolygon(point, zone.polygon))
      {
        unsigned int index = my * size_x_ + mx;
        mask_grid_[index] = zone.mask;
      }
    }
  }
  
  ROS_DEBUG("DirectionalLayer: Applied zone '%s' with mask 0x%02X", 
           zone.name.c_str(), zone.mask);
}

bool DirectionalLayer::pointInPolygon(const geometry_msgs::Point32& point,
                                     const std::vector<geometry_msgs::Point32>& polygon) const
{
  // Ray casting algorithm
  int n = polygon.size();
  bool inside = false;
  
  for (int i = 0, j = n - 1; i < n; j = i++)
  {
    const geometry_msgs::Point32& vi = polygon[i];
    const geometry_msgs::Point32& vj = polygon[j];
    
    if (((vi.y > point.y) != (vj.y > point.y)) &&
        (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x))
    {
      inside = !inside;
    }
  }
  
  return inside;
}

bool DirectionalLayer::worldToMask(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (resolution_ <= 0.0)
    return false;
    
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
  
  return (mx < size_x_ && my < size_y_);
}

void DirectionalLayer::maskToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

void DirectionalLayer::publishVisualization()
{
  if (!enable_visualization_ || !marker_pub_)
    return;

  boost::recursive_mutex::scoped_lock lock(lock_);
  
  visualization_msgs::MarkerArray marker_array;
  int marker_id = 0;

  // Clear previous markers
  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = visualization_frame_;
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.ns = "directional_zones";
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker.id = 0;
  marker_array.markers.push_back(clear_marker);

  // Create markers for each zone
  for (size_t i = 0; i < zones_.size(); ++i)
  {
    const DirectionalZoneData& zone = zones_[i];
    
    // Create polygon outline marker
    visualization_msgs::Marker polygon_marker = createPolygonMarker(zone, marker_id++);
    marker_array.markers.push_back(polygon_marker);
    
    // Create arrow markers for allowed directions
    std::vector<visualization_msgs::Marker> arrow_markers = createArrowMarkers(zone, marker_id);
    marker_id += 8;  // Reserve 8 IDs for arrows (one per direction)
    
    for (const auto& arrow : arrow_markers)
    {
      marker_array.markers.push_back(arrow);
    }
  }
  
  marker_pub_.publish(marker_array);
}

void DirectionalLayer::visualizationTimerCallback(const ros::TimerEvent& event)
{
  publishVisualization();
}

visualization_msgs::Marker DirectionalLayer::createPolygonMarker(const DirectionalZoneData& zone, int marker_id) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = visualization_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "directional_zones";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.05;  // Line width
  marker.color = getMaskColor(zone.mask);
  marker.color.a = 0.8;   // Semi-transparent
  
  marker.lifetime = ros::Duration(0);  // Persistent
  
  // Add polygon points
  for (const auto& point : zone.polygon)
  {
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.1;  // Slightly above ground
    marker.points.push_back(p);
  }
  
  // Close the polygon
  if (!zone.polygon.empty())
  {
    geometry_msgs::Point p;
    p.x = zone.polygon[0].x;
    p.y = zone.polygon[0].y;
    p.z = 0.1;
    marker.points.push_back(p);
  }
  
  return marker;
}

std::vector<visualization_msgs::Marker> DirectionalLayer::createArrowMarkers(const DirectionalZoneData& zone, int base_marker_id) const
{
  std::vector<visualization_msgs::Marker> arrows;
  
  // Get center point of the zone
  geometry_msgs::Point center = getCenterPoint(zone.polygon);
  
  // Create an arrow for each allowed direction
  for (uint8_t dir = 0; dir < 8; ++dir)
  {
    uint8_t bit_check = (1 << dir);
    bool is_allowed = (zone.mask & bit_check) != 0;
    
    ROS_DEBUG("DirectionalLayer: Zone '%s', dir=%d, bit_check=%d, mask=%d, allowed=%s", 
              zone.name.c_str(), dir, bit_check, zone.mask, is_allowed ? "YES" : "NO");
    
    if (!is_allowed)
      continue;  // Direction not allowed
      
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = visualization_frame_;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "directional_arrows";
    arrow.id = base_marker_id + dir;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    
    arrow.pose.position = center;
    arrow.pose.position.z = 0.2;  // Above the polygon
    
    // Set arrow orientation based on direction
    geometry_msgs::Vector3 dir_vec = getDirectionVector(dir);
    double yaw = atan2(dir_vec.y, dir_vec.x);
    
    // Debug output
    ROS_INFO("DirectionalLayer: Zone '%s' dir=%d, mask=%d, vec=(%.3f,%.3f), yaw=%.1f°", 
             zone.name.c_str(), dir, zone.mask, dir_vec.x, dir_vec.y, yaw * 180.0 / M_PI);
    
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = sin(yaw / 2.0);
    arrow.pose.orientation.w = cos(yaw / 2.0);
    
    // Set arrow size
    arrow.scale.x = arrow_scale_;      // Length
    arrow.scale.y = arrow_scale_ * 0.1; // Width
    arrow.scale.z = arrow_scale_ * 0.1; // Height
    
    // Set color based on direction
    std_msgs::ColorRGBA color;
    color.a = 0.9;
    switch (dir)
    {
      case 0: case 4: // E, W - Red
        color.r = 1.0; color.g = 0.0; color.b = 0.0; break;
      case 2: case 6: // N, S - Green  
        color.r = 0.0; color.g = 1.0; color.b = 0.0; break;
      case 1: case 3: case 5: case 7: // Diagonals - Blue
        color.r = 0.0; color.g = 0.0; color.b = 1.0; break;
    }
    arrow.color = color;
    
    arrow.lifetime = ros::Duration(0);  // Persistent
    
    arrows.push_back(arrow);
  }
  
  return arrows;
}

geometry_msgs::Point DirectionalLayer::getCenterPoint(const std::vector<geometry_msgs::Point32>& polygon) const
{
  geometry_msgs::Point center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;
  
  if (polygon.empty())
    return center;
    
  // Calculate centroid
  for (const auto& point : polygon)
  {
    center.x += point.x;
    center.y += point.y;
  }
  
  center.x /= polygon.size();
  center.y /= polygon.size();
  
  return center;
}

geometry_msgs::Vector3 DirectionalLayer::getDirectionVector(uint8_t direction) const
{
  geometry_msgs::Vector3 vec;
  vec.z = 0.0;
  
  // Map direction bits to unit vectors
  switch (direction)
  {
    case 0: vec.x = 1.0; vec.y = 0.0; break;   // E
    case 1: vec.x = 0.707; vec.y = 0.707; break;  // NE
    case 2: vec.x = 0.0; vec.y = 1.0; break;   // N
    case 3: vec.x = -0.707; vec.y = 0.707; break; // NW
    case 4: vec.x = -1.0; vec.y = 0.0; break;  // W
    case 5: vec.x = -0.707; vec.y = -0.707; break; // SW
    case 6: vec.x = 0.0; vec.y = -1.0; break;  // S
    case 7: vec.x = 0.707; vec.y = -0.707; break;  // SE
    default: vec.x = 0.0; vec.y = 0.0; break;
  }
  
  return vec;
}

std_msgs::ColorRGBA DirectionalLayer::getMaskColor(uint8_t mask) const
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  
  // Color based on number of allowed directions
  int allowed_directions = __builtin_popcount(mask);
  
  if (allowed_directions == 8)
  {
    // All directions - Green (no restriction)
    color.r = 0.0; color.g = 1.0; color.b = 0.0;
  }
  else if (allowed_directions == 0)
  {
    // No directions - Red (blocked)
    color.r = 1.0; color.g = 0.0; color.b = 0.0;
  }
  else if (allowed_directions <= 2)
  {
    // Very restricted - Orange
    color.r = 1.0; color.g = 0.5; color.b = 0.0;
  }
  else if (allowed_directions <= 4)
  {
    // Moderately restricted - Yellow
    color.r = 1.0; color.g = 1.0; color.b = 0.0;
  }
  else
  {
    // Slightly restricted - Light green
    color.r = 0.5; color.g = 1.0; color.b = 0.0;
  }
  
  return color;
}

}  // namespace costmap_2d
