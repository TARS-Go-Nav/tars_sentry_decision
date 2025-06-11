/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "behavior_node.hpp"

namespace tars_decision
{
/**
 * @brief Behavior tree class to initialize and execute the whole tree
 */
class BehaviorTree
{
  public:
    /**
     * @brief Constructor of BehaviorTree
     * @param root_node root node of the behavior tree
     * @param cycle_duration tick duration of the behavior tree (unit ms)
     */
    BehaviorTree(BehaviorNode::Ptr root_node) : root_node_(root_node) {}
    /**
     * @brief Loop to tick the behavior tree
     */
    void Run() {
        root_node_->Run();
    }

  private:
    //! root node of the behavior tree
    BehaviorNode::Ptr root_node_;
};

} // namespace tars_decision
