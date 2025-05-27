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
 *    Sanath Himasekhar
 *    Marko Mizdrak
 ********************************************************************************/

#include "trajectory_tracker_node.hpp"

using namespace std::chrono_literals;

namespace adore
{


TrajectoryTrackerNode::TrajectoryTrackerNode() :
  Node( "trajectory_tracker_node" )
{
  load_parameters();
  create_publishers();
  create_subscribers();
  initialize_controller();
}

void
TrajectoryTrackerNode::initialize_controller()
{
  switch( controller_type )
  {
    case 0:
      controller = controllers::NMPC();
      break;
    case 1:
      controller = controllers::PID();
      break;
    case 2:
      controller = controllers::iLQR();
      break;
    default:
      controller = controllers::PID();
      RCLCPP_ERROR( get_logger(), "Unknown controller type. Reverting to PID" );
      break;
  }

  controllers::set_parameters( controller, command_limits, controller_settings, model );
}

void
TrajectoryTrackerNode::load_parameters()
{
  std::string vehicle_model_file;
  declare_parameter( "vehicle_model_file", "" );
  get_parameter( "vehicle_model_file", vehicle_model_file );
  model = dynamics::PhysicalVehicleModel( vehicle_model_file, false );

  declare_parameter( "set_controller", 0 ); // default set to MPC
  get_parameter( "set_controller", controller_type );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );

  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  std::vector<std::string> keys;
  std::vector<double>      values;
  declare_parameter( "controller_settings_keys", keys );
  declare_parameter( "controller_settings_values", values );
  get_parameter( "controller_settings_keys", keys );
  get_parameter( "controller_settings_values", values );

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( get_logger(), "Controller settings keys and values size mismatch!" );
    return;
  }
  for( size_t i = 0; i < keys.size(); ++i )
  {
    controller_settings.insert( { keys[i], values[i] } );
  }

  std::vector<std::vector<double>> turn_polygon_values_list; // turn zone polyons
  std::string turn_polygons_file;
  declare_parameter( "turn_polygons_file", "" );
  get_parameter( "turn_polygons_file", turn_polygons_file );

  if (turn_polygons_file != "")
  {
    std::ifstream ifs( turn_polygons_file );
    if( !ifs.is_open() )
    {
      throw std::runtime_error( "Could not open file: " + turn_polygons_file ); 
    }
    nlohmann::json j;
    ifs >> j;

    // Convert the parameter into a Polygon2d
    for( const auto& turn_polygon_zone : j )
    {
      std::string label = turn_polygon_zone.at("label");
      const auto& points = turn_polygon_zone.at("polygon");
      if( points.size() >= 3 ) // minimum 3 x, 3 y
      {
        adore::math::Polygon2d polygon;
        for( const auto& point : points )
        {
          if( point.size() != 2 )
          {
            std::cerr << "invalied point size in polygon in the launch file" << std::endl;
            return;
          }
          double x = point[0];
          double y = point[1];
          polygon.points.push_back( { x, y } );
        }
        turn_indicator_zones[label].push_back( polygon );
      }
    }
  }
}

void
TrajectoryTrackerNode::create_publishers()
{
  publisher_vehicle_command          = create_publisher<adore_ros2_msgs::msg::VehicleCommand>( "next_vehicle_command", 1 );
  publisher_warning_indicator_lights = create_publisher<adore_ros2_msgs::msg::IndicatorState>( "FUN/IndicatorCommand", 1 );
  publisher_controller_trajectory    = create_publisher<adore_ros2_msgs::msg::Trajectory>( "controller_trajectory", 1 );
}

void
TrajectoryTrackerNode::create_subscribers()
{
  main_timer = create_wall_timer( 50ms, std::bind( &TrajectoryTrackerNode::timer_callback, this ) );

  subscriber_trajectory = create_subscription<adore_ros2_msgs::msg::Trajectory>( "trajectory_decision", 1,
                                                                                 std::bind( &TrajectoryTrackerNode::trajectory_callback,
                                                                                            this, std::placeholders::_1 ) );

  subscriber_vehicle_state = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 1, std::bind( &TrajectoryTrackerNode::vehicle_state_callback, this, std::placeholders::_1 ) );
}

void
TrajectoryTrackerNode::timer_callback()
{
  dynamics::VehicleCommand controls;

  // default to emergency
  controls.steering_angle = 0.0;
  controls.acceleration   = -2.0;

  // if( latest_vehicle_state )
  // {
  //   dynamics::integrate_up_to_time( *latest_vehicle_state, last_controls, now().seconds(), model.motion_model );
  // }


  if( !( latest_vehicle_state.has_value() && latest_trajectory.has_value() ) )
  {
    RCLCPP_INFO( get_logger(), "NO STATE OR NO TRAJECTORY" );
    indicators_on( true, true );
  }
  else if( ( latest_trajectory->label == "Emergency Stop" || latest_trajectory->label == "Requesting Assistance" ) )
  {
    indicators_on( true, true );
  }
  else if( latest_trajectory->label == "Standstill" )
  {
    controls.acceleration = -0.5;
    controls.steering_angle = last_controls.steering_angle;
    indicators_on( false, false );
  }
  else
  {
    auto next_controls = controllers::get_next_vehicle_command( controller, *latest_trajectory, latest_vehicle_state.value() );
    if( next_controls.has_value() )
      controls = next_controls.value();
    indicators_on( false, false ); // todo make work for turning
    check_turn_indicator_zones();

    if( auto* controller_ptr = std::get_if<controllers::iLQR>( &controller ) )
    {
      auto last_traj = controller_ptr->get_last_trajectory();
      publisher_controller_trajectory->publish( dynamics::conversions::to_ros_msg( last_traj ) );
    }
  }

  publisher_vehicle_command->publish( dynamics::conversions::to_ros_msg( controls ) );
  last_controls = controls;
}

void TrajectoryTrackerNode::check_turn_indicator_zones()
{
  if( !latest_vehicle_state )
    return;
  for( const auto& [label, polygons] : turn_indicator_zones )
  {
    for( const auto& polygon : polygons )
    {
      if( label == "Right" && polygon.point_inside( latest_vehicle_state.value() ) )
        indicators_on( false, true );
      if( label == "Left" && polygon.point_inside( latest_vehicle_state.value() ) )
        indicators_on( true, false );
    }
  }
}

void
TrajectoryTrackerNode::indicators_on( bool left, bool right )
{
  adore_ros2_msgs::msg::IndicatorState warning_indicator_lights_msg_to_send;
  warning_indicator_lights_msg_to_send.left_indicator_on  = left;
  warning_indicator_lights_msg_to_send.right_indicator_on = right;
  publisher_warning_indicator_lights->publish( warning_indicator_lights_msg_to_send );
}

void
TrajectoryTrackerNode::trajectory_callback( const adore_ros2_msgs::msg::Trajectory& msg )
{
  latest_trajectory = dynamics::conversions::to_cpp_type( msg );
}

void
TrajectoryTrackerNode::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  latest_vehicle_state = dynamics::conversions::to_cpp_type( msg );
}

} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::TrajectoryTrackerNode>() );
  rclcpp::shutdown();
  return 0;
}
