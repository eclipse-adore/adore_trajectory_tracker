/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Mikkel Skov Maarssø
 *    Sanath Himasekhar Konthala
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once
#include <cmath>
#include <math.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "adore_dynamics_conversions.hpp"
#include "adore_math/PiecewisePolynomial.h"
#include "adore_ros2_msgs/msg/indicator_state.hpp"
#include "adore_ros2_msgs/msg/trajectory.hpp"
#include "adore_ros2_msgs/msg/vehicle_command.hpp"

#include "OptiNLC_Data.h"
#include "OptiNLC_OCP.h"
#include "OptiNLC_Options.h"
#include "OptiNLC_Solver.h"
#include "controllers/controller.hpp"
#include "dynamics/integration.hpp"
#include "dynamics/physical_vehicle_model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace adore
{


class TrajectoryTrackerNode : public rclcpp::Node
{
private:

  /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
  rclcpp::TimerBase::SharedPtr                                       main_timer;
  rclcpp::Publisher<adore_ros2_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command;
  rclcpp::Publisher<adore_ros2_msgs::msg::IndicatorState>::SharedPtr publisher_warning_indicator_lights;
  rclcpp::Publisher<adore_ros2_msgs::msg::Trajectory>::SharedPtr     publisher_controller_trajectory;

  rclcpp::Subscription<adore_ros2_msgs::msg::Trajectory>::SharedPtr          subscriber_trajectory;
  rclcpp::Subscription<adore_ros2_msgs::msg::VehicleStateDynamic>::SharedPtr subscriber_vehicle_state;

  std::optional<dynamics::VehicleStateDynamic> latest_vehicle_state = std::nullopt;
  std::optional<dynamics::Trajectory>          latest_trajectory    = std::nullopt;

  dynamics::VehicleCommand last_controls;

  controllers::Controller controller;

  std::map<std::string, double> controller_settings;

  std::unordered_map<std::string, std::vector<math::Polygon2d>> turn_indicator_zones;

  dynamics::VehicleCommandLimits command_limits;

  int controller_type;

  dynamics::PhysicalVehicleModel model;

public:

  void indicators_on( bool left, bool right );
  void check_turn_indicator_zones();

    explicit TrajectoryTrackerNode(const rclcpp::NodeOptions & options);

  void load_parameters();
  void create_publishers();
  void create_subscribers();
  void initialize_controller();

  void timer_callback();

  void trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg );
  void vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg );
};
} // namespace adore
