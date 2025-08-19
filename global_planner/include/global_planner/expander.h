/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef _EXPANDER_H
#define _EXPANDER_H
#include <global_planner/potential_calculator.h>
#include <global_planner/planner_core.h>

// Forward declaration for DirectionalLayer
namespace costmap_2d {
    class DirectionalLayer;
}

namespace global_planner {

class Expander {
    public:
        Expander(PotentialCalculator* p_calc, int nx, int ny);
        virtual ~Expander() {}
        virtual bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) = 0;

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */
        void setLethalCost(unsigned char lethal_cost) {
            lethal_cost_ = lethal_cost;
        }
        void setNeutralCost(unsigned char neutral_cost) {
            neutral_cost_ = neutral_cost;
        }
        void setFactor(float factor) {
            factor_ = factor;
        }
        void setHasUnknown(bool unknown) {
            unknown_ = unknown;
        }
        
        // DirectionalLayer support
        void setDirectionalLayer(costmap_2d::DirectionalLayer* directional_layer) {
            directional_layer_ = directional_layer;
        }
        void setDirectionalPenaltyFactor(double penalty_factor) {
            directional_penalty_factor_ = penalty_factor;
        }
        void setOnewayStrictMode(bool strict_mode) {
            oneway_strict_mode_ = strict_mode;
        }
        void setDirectionalStrictMode(bool strict_mode) {
            oneway_strict_mode_ = strict_mode;
        }

        void clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s){
            int startCell = toIndex(gx, gy);
            for(int i=-s;i<=s;i++){
            for(int j=-s;j<=s;j++){
                int n = startCell+i+nx_*j;
                if(potential[n]<POT_HIGH)
                    continue;
                float c = costs[n]+neutral_cost_;
                float pot = p_calc_->calculatePotential(potential, c, n);
                potential[n] = pot;
            }
            }
        }
        
        // Check if movement direction is allowed by DirectionalLayer
        bool isDirectionAllowed(int from_x, int from_y, int to_x, int to_y);
        
        // Get directional penalty multiplier for movement
        float getDirectionalPenalty(int from_x, int from_y, int to_x, int to_y);
        
        // Check if direction should be treated as lethal (strict mode)
        bool isDirectionLethal(int from_x, int from_y, int to_x, int to_y);

    protected:
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
        bool unknown_;
        unsigned char lethal_cost_, neutral_cost_;
        int cells_visited_;
        float factor_;
        PotentialCalculator* p_calc_;
        
        // DirectionalLayer integration
        costmap_2d::DirectionalLayer* directional_layer_;
        double directional_penalty_factor_;
        bool oneway_strict_mode_;

};

} //end namespace global_planner
#endif
