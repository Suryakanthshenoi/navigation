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
#ifndef COSTMAP_2D_DIRECTIONAL_LAYER_H_
#define COSTMAP_2D_DIRECTIONAL_LAYER_H_

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/Empty.h>
#include <costmap_2d/DirectionalZone.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <memory>

namespace costmap_2d
{

/**
 * @struct DirectionalZoneData
 * @brief A zone with directional movement restrictions (internal data structure)
 */
struct DirectionalZoneData
{
  std::vector<geometry_msgs::Point32> polygon;  ///< Polygon vertices defining the zone
  uint8_t mask;                                 ///< 8-bit directional mask (0xFF = all directions allowed)
  std::string name;                             ///< Optional zone name for debugging
};

/**
 * @class DirectionalLayer
 * @brief A costmap layer that stores directional movement preferences for each grid cell
 * 
 * This layer does NOT modify the master costmap costs. Instead, it maintains
 * a parallel grid of 8-bit directional masks that can be queried by planners.
 * 
 * Directional masks use 8-connected neighbor encoding:
 * bit 0: E  (dx=+1, dy= 0)
 * bit 1: NE (dx=+1, dy=+1) 
 * bit 2: N  (dx= 0, dy=+1)
 * bit 3: NW (dx=-1, dy=+1)
 * bit 4: W  (dx=-1, dy= 0)
 * bit 5: SW (dx=-1, dy=-1)
 * bit 6: S  (dx= 0, dy=-1)
 * bit 7: SE (dx=+1, dy=-1)
 */
class DirectionalLayer : public Layer
{
public:
  DirectionalLayer();
  virtual ~DirectionalLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                          double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();
  virtual void reset();

  /**
   * @brief Get the directional mask for a specific grid cell
   * @param mx Grid x-coordinate 
   * @param my Grid y-coordinate
   * @return 8-bit directional mask (0xFF = all directions allowed, 0x00 = no movement allowed)
   */
  uint8_t getMask(unsigned int mx, unsigned int my) const;

  /**
   * @brief Check if movement in a specific direction is allowed from a grid cell
   * @param mx Grid x-coordinate
   * @param my Grid y-coordinate  
   * @param direction Direction bit (0-7 for 8-connected neighbors)
   * @return true if movement in that direction is allowed
   */
  bool isDirectionAllowed(unsigned int mx, unsigned int my, uint8_t direction) const;

  /**
   * @brief Get direction bit from delta coordinates
   * @param dx Change in x (-1, 0, or 1)
   * @param dy Change in y (-1, 0, or 1)
   * @return Direction bit (0-7), or 255 if invalid delta
   */
  static uint8_t getDirectionFromDelta(int dx, int dy);

  /**
   * @brief Publish visualization markers for directional zones
   */
  void publishVisualization();

  /**
   * @brief Timer callback for periodic visualization updates
   */
  void visualizationTimerCallback(const ros::TimerEvent& event);

private:
  /**
   * @brief Service callback to add directional zones
   */
  bool addZoneCallback(costmap_2d::DirectionalZone::Request& req, 
                      costmap_2d::DirectionalZone::Response& res);

  /**
   * @brief Service callback to clear all zones
   */
  bool clearZonesCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Load zones from parameter server
   */
  void loadZonesFromParams();

  /**
   * @brief Apply a directional zone to the mask grid
   * @param zone The zone to apply
   */
  void applyZone(const DirectionalZoneData& zone);

  /**
   * @brief Check if a point is inside a polygon using ray casting algorithm
   * @param point The point to test
   * @param polygon The polygon vertices
   * @return true if point is inside polygon
   */
  bool pointInPolygon(const geometry_msgs::Point32& point, 
                     const std::vector<geometry_msgs::Point32>& polygon) const;

  /**
   * @brief Convert world coordinates to mask grid coordinates
   * @param wx World x-coordinate
   * @param wy World y-coordinate
   * @param mx Output mask x-coordinate
   * @param my Output mask y-coordinate
   * @return true if conversion successful
   */
  bool worldToMask(double wx, double wy, unsigned int& mx, unsigned int& my) const;

  /**
   * @brief Convert mask grid coordinates to world coordinates
   * @param mx Mask x-coordinate
   * @param my Mask y-coordinate
   * @param wx Output world x-coordinate
   * @param wy Output world y-coordinate
   */
  void maskToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  /**
   * @brief Initialize the mask grid with default values
   */
  void initializeMaskGrid();

  /**
   * @brief Create polygon outline markers for a zone
   * @param zone The directional zone
   * @param marker_id Unique marker ID
   * @return Visualization marker for zone polygon
   */
  visualization_msgs::Marker createPolygonMarker(const DirectionalZoneData& zone, int marker_id) const;

  /**
   * @brief Create arrow markers showing allowed directions for a zone
   * @param zone The directional zone  
   * @param marker_id Base marker ID (arrows will use marker_id + direction_bit)
   * @return Vector of arrow markers for allowed directions
   */
  std::vector<visualization_msgs::Marker> createArrowMarkers(const DirectionalZoneData& zone, int marker_id) const;

  /**
   * @brief Get center point of a polygon
   * @param polygon The polygon vertices
   * @return Center point (centroid)
   */
  geometry_msgs::Point getCenterPoint(const std::vector<geometry_msgs::Point32>& polygon) const;

  /**
   * @brief Get arrow direction vector for a direction bit
   * @param direction Direction bit (0-7)
   * @return Unit vector pointing in that direction
   */
  geometry_msgs::Vector3 getDirectionVector(uint8_t direction) const;

  /**
   * @brief Get color for visualization based on mask value
   * @param mask Directional mask
   * @return RGBA color
   */
  std_msgs::ColorRGBA getMaskColor(uint8_t mask) const;

  // Mask grid storage
  std::vector<uint8_t> mask_grid_;  ///< Directional mask grid (parallel to costmap)
  unsigned int size_x_, size_y_;    ///< Grid dimensions
  double resolution_;               ///< Grid resolution (meters per cell)
  double origin_x_, origin_y_;      ///< Grid origin in world coordinates

  // Zone storage
  std::vector<DirectionalZoneData> zones_;  ///< List of directional zones
  
  // ROS interfaces
  ros::ServiceServer add_zone_srv_;   ///< Service to add zones
  ros::ServiceServer clear_zones_srv_; ///< Service to clear zones
  ros::Publisher marker_pub_;         ///< Publisher for visualization markers
  ros::Timer visualization_timer_;    ///< Timer for periodic visualization updates

  // Parameters
  std::string zones_file_;         ///< YAML file with zone definitions
  bool auto_load_zones_;          ///< Whether to automatically load zones from file
  uint8_t default_mask_;          ///< Default mask value for all cells (0xFF = all directions)
  bool enable_visualization_;     ///< Whether to publish visualization markers
  double visualization_rate_;     ///< Rate for publishing visualization markers (Hz)
  std::string visualization_frame_; ///< Frame ID for visualization markers
  double arrow_scale_;            ///< Scale factor for direction arrows

  // Thread safety
  mutable boost::recursive_mutex lock_;   ///< Mutex for thread-safe access to mask grid
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_DIRECTIONAL_LAYER_H_
