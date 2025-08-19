/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Navigation Team
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
 *********************************************************************/
#include <global_planner/expander.h>
#include <costmap_2d/directional_layer.h>

namespace global_planner {

// Initialize DirectionalLayer members in base constructor
Expander::Expander(PotentialCalculator* p_calc, int nx, int ny) :
        unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0), p_calc_(p_calc),
        directional_layer_(NULL), directional_penalty_factor_(2.0), oneway_strict_mode_(false) {
    setSize(nx, ny);
}

bool Expander::isDirectionAllowed(int from_x, int from_y, int to_x, int to_y) {
    if (!directional_layer_) {
        return true;  // No directional constraints
    }
    
    // Calculate movement direction
    int dx = to_x - from_x;
    int dy = to_y - from_y;
    
    // Get directional mask at starting position
    uint8_t mask = directional_layer_->getMask(from_x, from_y);
    
    // Map movement delta to direction bit
    uint8_t direction_bit = 0;
    if (dx == 1 && dy == 0)       direction_bit = 0;  // East
    else if (dx == 1 && dy == 1)  direction_bit = 1;  // Northeast  
    else if (dx == 0 && dy == 1)  direction_bit = 2;  // North
    else if (dx == -1 && dy == 1) direction_bit = 3;  // Northwest
    else if (dx == -1 && dy == 0) direction_bit = 4;  // West
    else if (dx == -1 && dy == -1) direction_bit = 5; // Southwest
    else if (dx == 0 && dy == -1) direction_bit = 6;  // South
    else if (dx == 1 && dy == -1) direction_bit = 7;  // Southeast
    else return true;  // Invalid movement, let default logic handle
    
    // Check if direction is allowed
    return (mask & (1 << direction_bit)) != 0;
}

float Expander::getDirectionalPenalty(int from_x, int from_y, int to_x, int to_y) {
    if (!directional_layer_) {
        return 1.0;  // No penalty
    }
    
    if (isDirectionAllowed(from_x, from_y, to_x, to_y)) {
        return 1.0;  // No penalty for allowed directions
    } else {
        // In strict mode, treat as lethal obstacle
        if (oneway_strict_mode_) {
            return static_cast<float>(lethal_cost_);  // Return lethal cost to block completely
        } else {
            // Apply normal penalty for disallowed directions
            return static_cast<float>(directional_penalty_factor_);
        }
    }
}

// New method to check if movement should be treated as lethal
bool Expander::isDirectionLethal(int from_x, int from_y, int to_x, int to_y) {
    if (!directional_layer_ || !oneway_strict_mode_) {
        return false;  // Not lethal in non-strict mode
    }
    
    return !isDirectionAllowed(from_x, from_y, to_x, to_y);
}

}  // namespace global_planner
