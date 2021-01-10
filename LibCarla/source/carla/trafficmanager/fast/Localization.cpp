#include "carla/trafficmanager/fast/Localization.h"
#include "carla/trafficmanager/fast/TrafficManagerLocal.h"
#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/LocalizationUtils.h"


namespace carla {
namespace traffic_manager {

using namespace constants::PathBufferUpdate;
using namespace constants::LaneChange;
using namespace constants::WaypointSelection;

FastLocalizationStage::FastLocalizationStage(FastALSM & alsm, LocalMapPtr local_map):alsm(alsm), local_map(local_map) {
}

void FastLocalizationStage::Update(ActorId actor_id,
                                   const ActorParameters & parameters,
                                   const GlobalParameters & global_parameters) {
  auto & state = alsm.GetState(actor_id);

  // Update the kinematic_state
  const auto & kinematic_state = state.kinematic_state;
  const cg::Location & vehicle_location = kinematic_state.location;
  const cg::Vector3D & heading_vector = kinematic_state.rotation.GetForwardVector();
  const cg::Vector3D & vehicle_velocity_vector = kinematic_state.velocity;
  const float vehicle_speed = vehicle_velocity_vector.Length();

  // Speed dependent waypoint horizon length.
  float horizon_length = std::min(vehicle_speed * HORIZON_RATE + MINIMUM_HORIZON_LENGTH, MAXIMUM_HORIZON_LENGTH);
  const float horizon_square = SQUARE(horizon_length);

  Buffer &waypoint_buffer = state.buffer;

  // Clear buffer if vehicle is too far from the first waypoint in the buffer.
  if (!waypoint_buffer.empty() &&
      cg::Math::DistanceSquared(waypoint_buffer.front()->GetLocation(),
                                vehicle_location) > SQUARE(MAX_START_DISTANCE)) {

    auto number_of_pops = waypoint_buffer.size();
    for (uint64_t j = 0u; j < number_of_pops; ++j) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
    }
  }

  bool is_at_junction_entrance = false;

  if (!waypoint_buffer.empty()) {
    // Purge passed waypoints.
    float dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
    while (dot_product <= 0.0f && !waypoint_buffer.empty()) {

      PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      if (!waypoint_buffer.empty()) {
        dot_product = DeviationDotProduct(vehicle_location, heading_vector, waypoint_buffer.front()->GetLocation());
      }
    }

    if (!waypoint_buffer.empty()) {
      // Determine if the vehicle is at the entrance of a junction.
      SimpleWaypointPtr look_ahead_point = GetTargetWaypoint(waypoint_buffer, JUNCTION_LOOK_AHEAD).first;
      is_at_junction_entrance = !waypoint_buffer.front()->CheckJunction() && look_ahead_point->CheckJunction();
      if (is_at_junction_entrance
          // Exception for roundabout in Town03.
          && local_map->GetMapName() == "Town03"
          && vehicle_location.SquaredLength() < SQUARE(30)) {
        is_at_junction_entrance = false;
      }
    }

    // Purge waypoints too far from the front of the buffer.
    while (!is_at_junction_entrance
           && !waypoint_buffer.empty()
           && waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) > horizon_square) {
      PopWaypoint(actor_id, track_traffic, waypoint_buffer, false);
    }
  }

  // Initializing buffer if it is empty.
  if (waypoint_buffer.empty()) {
    SimpleWaypointPtr closest_waypoint = local_map->GetWaypoint(vehicle_location);
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, closest_waypoint);
  }

  // Assign a lane change.
  const ChangeLaneInfo lane_change_info = parameters.force_lane_change;
  bool force_lane_change = lane_change_info.change_lane;
  bool lane_change_direction = lane_change_info.direction;

  if (!force_lane_change) {
    float perc_keep_right = parameters.percentage_keep_right;
    if (perc_keep_right >= 0.0f && perc_keep_right >= state.random_device.next()) {
      force_lane_change = true;
      lane_change_direction = true;
    }
  }

  const SimpleWaypointPtr front_waypoint = waypoint_buffer.front();
  const float lane_change_distance = SQUARE(std::max(10.0f * vehicle_speed, INTER_LANE_CHANGE_DISTANCE));

  bool recently_not_executed_lane_change = last_lane_change_location.find(actor_id) == last_lane_change_location.end();
  bool done_with_previous_lane_change = true;
  if (!recently_not_executed_lane_change) {
    float distance_frm_previous = cg::Math::DistanceSquared(last_lane_change_location.at(actor_id), vehicle_location);
    done_with_previous_lane_change = distance_frm_previous > lane_change_distance;
  }
  bool auto_or_force_lane_change = parameters.auto_lane_change || force_lane_change;
  bool front_waypoint_not_junction = !front_waypoint->CheckJunction();

  if (auto_or_force_lane_change
      && front_waypoint_not_junction
      && (recently_not_executed_lane_change || done_with_previous_lane_change)) {

    SimpleWaypointPtr change_over_point = AssignLaneChange(state, vehicle_location, vehicle_speed,
                                                           force_lane_change, lane_change_direction);

    if (change_over_point != nullptr) {
      if (last_lane_change_location.find(actor_id) != last_lane_change_location.end()) {
        last_lane_change_location.at(actor_id) = vehicle_location;
      } else {
        last_lane_change_location.insert({actor_id, vehicle_location});
      }
      auto number_of_pops = waypoint_buffer.size();
      for (uint64_t j = 0u; j < number_of_pops; ++j) {
        PopWaypoint(actor_id, track_traffic, waypoint_buffer);
      }
      PushWaypoint(actor_id, track_traffic, waypoint_buffer, change_over_point);
    }
  }

  // Populating the buffer.
  while (waypoint_buffer.back()->DistanceSquared(waypoint_buffer.front()) <= horizon_square) {

    SimpleWaypointPtr furthest_waypoint = waypoint_buffer.back();
    std::vector<SimpleWaypointPtr> next_waypoints = furthest_waypoint->GetNextWaypoint();
    uint64_t selection_index = 0u;
    // Pseudo-randomized path selection if found more than one choice.
    if (next_waypoints.size() > 1) {
      // Arranging selection points from right to left.
      std::sort(next_waypoints.begin(), next_waypoints.end(),
                [&furthest_waypoint](const SimpleWaypointPtr &a, const SimpleWaypointPtr &b) {
                  float a_x_product = DeviationCrossProduct(furthest_waypoint->GetLocation(),
                                                            furthest_waypoint->GetForwardVector(),
                                                            a->GetLocation());
                  float b_x_product = DeviationCrossProduct(furthest_waypoint->GetLocation(),
                                                            furthest_waypoint->GetForwardVector(),
                                                            b->GetLocation());
                  return a_x_product < b_x_product;
                });
      double r_sample = state.random_device.next();
      double s_bucket = 100.0 / next_waypoints.size();
      selection_index = static_cast<uint64_t>(std::floor(r_sample/s_bucket));
    } else if (next_waypoints.size() == 0) {
      if (!global_parameters.osm_mode) {
        std::cout << "This map has dead-end roads, please change the set_open_street_map parameter to true" << std::endl;
      }
      state.remove = true;
      break;
    }
    SimpleWaypointPtr next_wp_selection = next_waypoints.at(selection_index);
    PushWaypoint(actor_id, track_traffic, waypoint_buffer, next_wp_selection);
  }

  ExtendAndFindSafeSpace(state, is_at_junction_entrance, waypoint_buffer);

  // Editing output array
  state.localization.is_at_junction_entrance = is_at_junction_entrance;

  if (is_at_junction_entrance) {
    const SimpleWaypointPair &safe_space_end_points = vehicles_at_junction_entrance.at(actor_id);
    state.localization.junction_end_point = safe_space_end_points.first;
    state.localization.safe_point = safe_space_end_points.second;
  } else {
    state.localization.junction_end_point = nullptr;
    state.localization.safe_point = nullptr;
  }

  // Updating geodesic grid position for actor.
  track_traffic.UpdateGridPosition(actor_id, waypoint_buffer);
}


void FastLocalizationStage::ExtendAndFindSafeSpace(const ActorState & state,
                                                   const bool is_at_junction_entrance,
                                                   Buffer &waypoint_buffer) {

  SimpleWaypointPtr junction_end_point = nullptr;
  SimpleWaypointPtr safe_point_after_junction = nullptr;

  if (is_at_junction_entrance
      && vehicles_at_junction_entrance.find(state.id) == vehicles_at_junction_entrance.end()) {

    bool entered_junction = false;
    bool past_junction = false;
    bool safe_point_found = false;
    SimpleWaypointPtr current_waypoint = nullptr;
    SimpleWaypointPtr junction_begin_point = nullptr;
    float safe_distance_squared = SQUARE(SAFE_DISTANCE_AFTER_JUNCTION);

    // Scanning existing buffer points.
    for (unsigned long i = 0u; i < waypoint_buffer.size() && !safe_point_found; ++i) {
      current_waypoint = waypoint_buffer.at(i);
      if (!entered_junction && current_waypoint->CheckJunction()) {
        entered_junction = true;
        junction_begin_point = current_waypoint;
      }
      if (entered_junction && !past_junction && !current_waypoint->CheckJunction()) {
        past_junction = true;
        junction_end_point = current_waypoint;
      }
      if (past_junction && junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared) {
        safe_point_found = true;
        safe_point_after_junction = current_waypoint;
      }
    }

    // Extend buffer if safe point not found.
    if (!safe_point_found) {
      bool abort = false;

      while (!past_junction && !abort) {
        NodeList next_waypoints = current_waypoint->GetNextWaypoint();
        if (!next_waypoints.empty()) {
          current_waypoint = next_waypoints.front();
          PushWaypoint(state.id, track_traffic, waypoint_buffer, current_waypoint);
          if (!current_waypoint->CheckJunction()) {
            past_junction = true;
            junction_end_point = current_waypoint;
          }
        } else {
          abort = true;
        }
      }

      while (!safe_point_found && !abort) {
        std::vector<SimpleWaypointPtr> next_waypoints = current_waypoint->GetNextWaypoint();
        if ((junction_end_point->DistanceSquared(current_waypoint) > safe_distance_squared)
            || next_waypoints.size() > 1
            || current_waypoint->CheckJunction()) {

          safe_point_found = true;
          safe_point_after_junction = current_waypoint;
        } else {
          if (!next_waypoints.empty()) {
            current_waypoint = next_waypoints.front();
            PushWaypoint(state.id, track_traffic, waypoint_buffer, current_waypoint);
          } else {
            abort = true;
          }
        }
      }
    }

    if (junction_end_point != nullptr &&
        safe_point_after_junction != nullptr &&
        junction_begin_point->DistanceSquared(junction_end_point) < SQUARE(MIN_JUNCTION_LENGTH)) {

      junction_end_point = nullptr;
      safe_point_after_junction = nullptr;
    }

    vehicles_at_junction_entrance.insert({state.id, {junction_end_point, safe_point_after_junction}});
  }
  else if (!is_at_junction_entrance
           && vehicles_at_junction_entrance.find(state.id) != vehicles_at_junction_entrance.end()) {

    vehicles_at_junction_entrance.erase(state.id);
  }
}

//void FastLocalizationStage::RemoveActor(ActorId actor_id) {
//  TODO:
//  last_lane_change_location.erase(actor_id);
//}

SimpleWaypointPtr FastLocalizationStage::AssignLaneChange(const ActorState & state,
                                                          const cg::Location vehicle_location,
                                                          const float vehicle_speed,
                                                          bool force, bool direction) {

  // Waypoint representing the new starting point for the waypoint buffer
  // due to lane change. Remains nullptr if lane change not viable.
  SimpleWaypointPtr change_over_point = nullptr;

  // Retrieve waypoint buffer for current vehicle.
  const Buffer &waypoint_buffer = state.buffer;

  // Check buffer is not empty.
  if (!waypoint_buffer.empty()) {
    // Get the left and right waypoints for the current closest waypoint.
    const SimpleWaypointPtr &current_waypoint = waypoint_buffer.front();
    const SimpleWaypointPtr left_waypoint = current_waypoint->GetLeftWaypoint();
    const SimpleWaypointPtr right_waypoint = current_waypoint->GetRightWaypoint();

    // Retrieve vehicles with overlapping waypoint buffers with current vehicle.
    const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(state.id);

    // Find immediate in-lane obstacle and check if any are too close to initiate lane change.
    bool obstacle_too_close = false;
    float minimum_squared_distance = std::numeric_limits<float>::infinity();
    ActorId obstacle_actor_id = 0u;
    for (auto i = blocking_vehicles.begin();
         i != blocking_vehicles.end() && !obstacle_too_close && !force;
         ++i) {
      const ActorId &other_actor_id = *i;
      // Find vehicle in buffer map and check if it's buffer is not empty.

      const ActorState & other_state = alsm.GetState(other_actor_id);
      if (other_state.alive && other_state.registered && !other_state.buffer.empty()) {
        const Buffer &other_buffer = other_state.buffer;
        const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
        const cg::Location other_location = other_current_waypoint->GetLocation();

        const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
        cg::Vector3D reference_to_other = other_location - current_waypoint->GetLocation();
        const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

        WaypointPtr current_raw_waypoint = current_waypoint->GetWaypoint();
        WaypointPtr other_current_raw_waypoint = other_current_waypoint->GetWaypoint();
        // Check both vehicles are not in junction,
        // Check if the other vehicle is in front of the current vehicle,
        // Check if the two vehicles have acceptable angular deviation between their headings.
        if (!current_waypoint->CheckJunction()
            && !other_current_waypoint->CheckJunction()
            && other_current_raw_waypoint->GetRoadId() == current_raw_waypoint->GetRoadId()
            && other_current_raw_waypoint->GetLaneId() == current_raw_waypoint->GetLaneId()
            && cg::Math::Dot(reference_heading, reference_to_other) > 0.0f
            && cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE) {
          float squared_distance = cg::Math::DistanceSquared(vehicle_location, other_location);
          // Abort if the obstacle is too close.
          if (squared_distance > SQUARE(MINIMUM_LANE_CHANGE_DISTANCE)) {
            // Remember if the new vehicle is closer.
            if (squared_distance < minimum_squared_distance && squared_distance < SQUARE(MAXIMUM_LANE_OBSTACLE_DISTANCE)) {
              minimum_squared_distance = squared_distance;
              obstacle_actor_id = other_actor_id;
            }
          } else {
            obstacle_too_close = true;
          }
        }
      }
    }

    // If a valid immediate obstacle found.
    if (!obstacle_too_close && obstacle_actor_id != 0u && !force) {
      const Buffer &other_buffer = alsm.GetState(obstacle_actor_id).buffer;
      const SimpleWaypointPtr &other_current_waypoint = other_buffer.front();
      const auto other_neighbouring_lanes = {other_current_waypoint->GetLeftWaypoint(),
                                             other_current_waypoint->GetRightWaypoint()};

      // Flags reflecting whether adjacent lanes are free near the obstacle.
      bool distant_left_lane_free = false;
      bool distant_right_lane_free = false;

      // Check if the neighbouring lanes near the obstructing vehicle are free of other vehicles.
      bool left_right = true;
      for (auto &candidate_lane_wp : other_neighbouring_lanes) {
        if (candidate_lane_wp != nullptr &&
            track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0) {

          if (left_right)
            distant_left_lane_free = true;
          else
            distant_right_lane_free = true;
        }
        left_right = !left_right;
      }

      // Based on what lanes are free near the obstacle,
      // find the change over point with no vehicles passing through them.
      if (distant_right_lane_free && right_waypoint != nullptr
          && track_traffic.GetPassingVehicles(right_waypoint->GetId()).size() == 0) {
        change_over_point = right_waypoint;
      } else if (distant_left_lane_free && left_waypoint != nullptr
                 && track_traffic.GetPassingVehicles(left_waypoint->GetId()).size() == 0) {
        change_over_point = left_waypoint;
      }
    } else if (force) {
      if (direction && right_waypoint != nullptr) {
        change_over_point = right_waypoint;
      } else if (!direction && left_waypoint != nullptr) {
        change_over_point = left_waypoint;
      }
    }

    if (change_over_point != nullptr) {
      const float change_over_distance = cg::Math::Clamp(1.5f * vehicle_speed, 3.0f, 20.0f);
      const auto starting_point = change_over_point;
      while (change_over_point->DistanceSquared(starting_point) < SQUARE(change_over_distance) &&
             !change_over_point->CheckJunction()) {
        change_over_point = change_over_point->GetNextWaypoint()[0];
      }
    }
  }

  return change_over_point;
}

} // namespace traffic_manager
} // namespace carla
