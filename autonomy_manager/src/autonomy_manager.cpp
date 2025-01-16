//
// Created by aljoscha on 20.02.24.
//

#include "../include/autonomy_manager/autonomy_manager.h"

namespace autonomy_manager {} // autonomy_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE( autonomy_manager::AutonomyManager );