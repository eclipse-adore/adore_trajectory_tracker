/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "trajectory_tracker.hpp"

using namespace std::chrono_literals;

namespace adore
{


TrajectoryTracker::TrajectoryTracker( const rclcpp::NodeOptions& options ) :
  Node( "trajectory_tracker_node", options )
{
  load_parameters();
  create_publishers();
  create_subscribers();
  initialize_controller();
  RCLCPP_INFO( get_logger(), "TrajectoryTrackerNinitialized succesfully." );
}

void
TrajectoryTracker::initialize_controller()
{
  set_controller_type();
  controllers::set_parameters( controller, controller_settings, model );
}

void 
TrajectoryTracker::set_controller_type()
{
  if ( controller_type == "MPC")
  {
      controller = controllers::PurePursuit();
      RCLCPP_INFO( get_logger(), "Using Pure Pursuit controller." );
      return;
  }

  if ( controller_type == "PID")
  {
      controller = controllers::PID();
      RCLCPP_INFO( get_logger(), "Using PID controller." );
      return;
  }

  if ( controller_type == "iLQR")
  {
      controller = controllers::iLQR();
      RCLCPP_INFO( get_logger(), "Using iLQR controller." );
      return;
  }

  if ( controller_type == "MPC")
  {
      controller = controllers::PassThrough();
      RCLCPP_INFO( get_logger(), "Using Pure Pursuit controller." );
      return;
  }

  if ( controller_type == "Passthrough")
  {
      controller = controllers::PassThrough();
      RCLCPP_INFO( get_logger(), "Using Passthrough controller." );
      return;
  }

  controller = controllers::PassThrough();
  RCLCPP_ERROR( get_logger(), "Unknown controller type. Reverting to Passthrough" );
}

void
TrajectoryTracker::load_parameters()
{
  std::string vehicle_model_file = declare_parameter( "vehicle_model_file", "" );
  model                          = dynamics::PhysicalVehicleModel( vehicle_model_file, false );

  controller_type = declare_parameter<std::string>( "set_controller", "MPC" ); // default set to MPC

  std::vector<std::string> keys   = declare_parameter( "controller_settings_keys", std::vector<std::string>{} );
  std::vector<double>      values = declare_parameter( "controller_settings_values", std::vector<double>{} );

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( get_logger(), "Controller settings keys and values size mismatch!" );
    return;
  }
  for( size_t i = 0; i < keys.size(); ++i )
  {
    controller_settings.insert( { keys[i], values[i] } );
  }
}

void
TrajectoryTracker::create_publishers()
{
  publisher_vehicle_command          = create_publisher<VehicleCommandAdapter>( "next_vehicle_command", 1 );
  publisher_warning_indicator_lights = create_publisher<adore_ros2_msgs::msg::IndicatorState>( "FUN/IndicatorCommand", 1 );
  publisher_controller_trajectory    = create_publisher<TrajectoryAdapter>( "controller_trajectory", 1 );
}

void
TrajectoryTracker::create_subscribers()
{
  main_timer = create_wall_timer( 50ms, std::bind( &TrajectoryTracker::timer_callback, this ) );

  subscriber_trajectory = create_subscription<TrajectoryAdapter>( "trajectory_decision", 1,
                                                                  std::bind( &TrajectoryTracker::trajectory_callback, this,
                                                                             std::placeholders::_1 ) );

  subscriber_vehicle_state = create_subscription<StateAdapter>( "vehicle_state_dynamic", 1,
                                                                std::bind( &TrajectoryTracker::vehicle_state_callback, this,
                                                                           std::placeholders::_1 ) );
}

void
TrajectoryTracker::timer_callback()
{
  dynamics::VehicleCommand controls{};
  constexpr double         emergency_accel  = -2.0;
  constexpr double         standstill_accel = -0.5;

  // Default to emergency
  controls.steering_angle = 0.0;
  controls.acceleration   = emergency_accel;

  if( latest_vehicle_state )
  {
    dynamics::integrate_up_to_time( *latest_vehicle_state, last_controls, now().seconds(), model.motion_model );
  }

  if( latest_vehicle_state && latest_trajectory )
  {
    const auto& label = latest_trajectory->label;

    if( label == "waiting for mission" )
    {
      controls.acceleration = standstill_accel;
    }
    else if( label != "emergency stop" && label != "remote operations (waiting for remote operator instructions)" )
    {
      auto next_controls = controllers::get_next_vehicle_command( controller, *latest_trajectory, *latest_vehicle_state );
      if( next_controls )
        controls = *next_controls;

      auto last_traj = controllers::get_last_trajectory( controller );
      publisher_controller_trajectory->publish( last_traj );
    }
  }
  update_blinker_state();
  publisher_vehicle_command->publish( controls );
  last_controls = controls;
}

void
TrajectoryTracker::update_blinker_state()
{
  if( !latest_vehicle_state || !latest_trajectory )
  {
    indicators_on( true, true );
    return;
  }
  const auto& label = latest_trajectory->label;
  if( label == "Emergency Stop" || label == "Requesting Assistance" )
  {
    indicators_on( true, true );
    return;
  }
  indicators_on( false, false );
}

void
TrajectoryTracker::indicators_on( bool left, bool right )
{
  adore_ros2_msgs::msg::IndicatorState warning_indicator_lights_msg_to_send;
  warning_indicator_lights_msg_to_send.left_indicator_on  = left;
  warning_indicator_lights_msg_to_send.right_indicator_on = right;
  publisher_warning_indicator_lights->publish( warning_indicator_lights_msg_to_send );
}

void
TrajectoryTracker::trajectory_callback( const dynamics::Trajectory& trajectory )
{
  latest_trajectory = trajectory;
}

void
TrajectoryTracker::vehicle_state_callback( const dynamics::VehicleStateDynamic& state )
{
  latest_vehicle_state = state;
}

} // namespace adore

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::TrajectoryTracker)
