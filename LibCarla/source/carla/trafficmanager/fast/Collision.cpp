
#include "carla/geom/Math.h"

#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/LocalizationUtils.h"

#include "carla/trafficmanager/fast/Collision.h"
#include "carla/trafficmanager/fast/TrafficManagerLocal.h"

namespace carla {
namespace traffic_manager {

using Point2D = bg::model::point<double, 2, bg::cs::cartesian>;
using TLS = carla::rpc::TrafficLightState;

using namespace constants::Collision;
using constants::WaypointSelection::JUNCTION_LOOK_AHEAD;

FastCollisionStage::FastCollisionStage(FastALSM & alsm, const TrackTraffic & track_traffic):alsm(alsm),
  track_traffic(track_traffic) {}

static cg::Vector3D GetDimensions(const ActorState & actor) {
  const StaticAttributes &attributes = actor.static_attributes;
  return cg::Vector3D(attributes.half_length, attributes.half_width, attributes.half_height);

}

void FastCollisionStage::Update(ActorId ego_actor_id,
                                const ActorParameters &parameters,
                                const GlobalParameters &global_parameters) {
  auto & state = alsm.GetState(ego_actor_id);

  if (state.registered && state.alive) {
    state.collision = {std::numeric_limits<float>::infinity(), 0, false};

    const auto & kinematic_state = state.kinematic_state;

    const cg::Location ego_location = kinematic_state.location;
    const Buffer &ego_buffer = state.buffer;
    const unsigned long look_ahead_index = GetTargetWaypoint(ego_buffer, JUNCTION_LOOK_AHEAD).second;

    ActorIdSet overlapping_actors = track_traffic.GetOverlappingVehicles(ego_actor_id);
    std::vector<std::pair<double, ActorId>> collision_candidate_ids;

    // Run through vehicles with overlapping paths and filter them;
    float collision_radius_square = SQUARE(MAX_COLLISION_RADIUS);
    for (ActorId overlapping_actor_id : overlapping_actors) {
      if (overlapping_actor_id != ego_actor_id) {
        // If actor is within maximum collision avoidance and vertical overlap range.
        const cg::Location &overlapping_actor_location = alsm.GetState(overlapping_actor_id).kinematic_state.location;
        double dist = cg::Math::DistanceSquared(overlapping_actor_location, ego_location);
        if (dist < collision_radius_square
            && std::abs(ego_location.z - overlapping_actor_location.z) < VERTICAL_OVERLAP_THRESHOLD) {
          collision_candidate_ids.push_back(std::make_pair(dist, overlapping_actor_id));
        }
      }
    }

    // Sorting collision candidates in accending order of distance to current vehicle.
    std::sort(collision_candidate_ids.begin(), collision_candidate_ids.end());

    // Check every actor in the vicinity if it poses a collision hazard.
    for (const auto & i: collision_candidate_ids) {
      const ActorId other_actor_id = i.second;
      const auto & other = alsm.GetState(other_actor_id);
      if (other.alive && parameters.CanCollide(other_actor_id)) {
        std::pair<bool, float> negotiation_result = NegotiateCollision(state, other, look_ahead_index);
        if (negotiation_result.first) {
          float percentage_ignore = -1.f;
          if (other.static_attributes.actor_type == ActorType::Vehicle)
            percentage_ignore = parameters.percentage_ignore_vehicles;
          if (other.static_attributes.actor_type == ActorType::Pedestrian)
            percentage_ignore = parameters.percentage_ignore_walkers;
          if (percentage_ignore <= state.random_device.next()) {
            state.collision = {negotiation_result.second, other_actor_id, true};
            break;
          }
        }
      }
    }
  }
}

float FastCollisionStage::GetBoundingBoxExtention(const ActorState & actor) {

  const float velocity = cg::Math::Dot(actor.kinematic_state.velocity,
                                       actor.kinematic_state.rotation.GetForwardVector());
  float bbox_extension;
  // Using a linear function to calculate boundary length.
  bbox_extension = BOUNDARY_EXTENSION_RATE * velocity + BOUNDARY_EXTENSION_MINIMUM;
  // If a valid collision lock present, change boundary length to maintain lock.
  if (actor.collision_lock.lead_vehicle_id > 0){
    const auto &lock = actor.collision_lock;
    float lock_boundary_length = static_cast<float>(lock.distance_to_lead_vehicle + LOCKING_DISTANCE_PADDING);
    // Only extend boundary track vehicle if the leading vehicle
    // if it is not further than velocity dependent extension by MAX_LOCKING_EXTENSION.
    if ((lock_boundary_length - lock.initial_lock_distance) < MAX_LOCKING_EXTENSION) {
      bbox_extension = lock_boundary_length;
    }
  }

  return bbox_extension;
}

LocationVector FastCollisionStage::GetBoundary(const ActorState & actor) {
  const cg::Vector3D heading_vector = actor.kinematic_state.rotation.GetForwardVector();

  float forward_extension = 0.0f;
  if (actor.static_attributes.actor_type == ActorType::Pedestrian) {
    // Extend the pedestrians bbox to "predict" where they'll be and avoid collisions.
    forward_extension = actor.kinematic_state.velocity.Length() * WALKER_TIME_EXTENSION;
  }

  cg::Vector3D dimensions = GetDimensions(actor);

  float bbox_x = dimensions.x;
  float bbox_y = dimensions.y;

  const cg::Vector3D x_boundary_vector = heading_vector * (bbox_x + forward_extension);
  const auto perpendicular_vector = cg::Vector3D(-heading_vector.y, heading_vector.x, 0.0f).MakeSafeUnitVector(EPSILON);
  const cg::Vector3D y_boundary_vector = perpendicular_vector * (bbox_y + forward_extension);

  // Four corners of the vehicle in top view clockwise order (left-handed system).
  const cg::Location & location = actor.kinematic_state.location;
  LocationVector bbox_boundary = {
          location + cg::Location(x_boundary_vector - y_boundary_vector),
          location + cg::Location(-1.0f * x_boundary_vector - y_boundary_vector),
          location + cg::Location(-1.0f * x_boundary_vector + y_boundary_vector),
          location + cg::Location(x_boundary_vector + y_boundary_vector),
  };

  return bbox_boundary;
}

LocationVector FastCollisionStage::GetGeodesicBoundary(const ActorState & actor) {
  LocationVector geodesic_boundary;

  if (geodesic_boundary_map.find(actor.id) != geodesic_boundary_map.end()) {
    geodesic_boundary = geodesic_boundary_map.at(actor.id);
  } else {
    const LocationVector bbox = GetBoundary(actor);

    if (actor.registered && actor.buffer.size()) {

      float bbox_extension = GetBoundingBoxExtention(actor);
      const float specific_lead_distance = 2.f; // TODO: parameters.GetDistanceToLeadingVehicle(actor_id);
      bbox_extension = std::max(specific_lead_distance, bbox_extension);
      const float bbox_extension_square = SQUARE(bbox_extension);

      LocationVector left_boundary;
      LocationVector right_boundary;
      cg::Vector3D dimensions = GetDimensions(actor);
      const float width = dimensions.y;
      const float length = dimensions.x;

      const Buffer &waypoint_buffer = actor.buffer;
      const TargetWPInfo target_wp_info = GetTargetWaypoint(waypoint_buffer, length);
      const SimpleWaypointPtr boundary_start = target_wp_info.first;
      const uint64_t boundary_start_index = target_wp_info.second;

      // At non-signalized junctions, we extend the boundary across the junction
      // and in all other situations, boundary length is velocity-dependent.
      SimpleWaypointPtr boundary_end = nullptr;
      SimpleWaypointPtr current_point = waypoint_buffer.at(boundary_start_index);
      bool reached_distance = false;
      for (uint64_t j = boundary_start_index; !reached_distance && (j < waypoint_buffer.size()); ++j) {
        if (boundary_start->DistanceSquared(current_point) > bbox_extension_square || j == waypoint_buffer.size() - 1) {
          reached_distance = true;
        }
        if (boundary_end == nullptr
            || cg::Math::Dot(boundary_end->GetForwardVector(), current_point->GetForwardVector()) < COS_10_DEGREES
            || reached_distance) {

          const cg::Vector3D heading_vector = current_point->GetForwardVector();
          const cg::Location location = current_point->GetLocation();
          cg::Vector3D perpendicular_vector = cg::Vector3D(-heading_vector.y, heading_vector.x, 0.0f);
          perpendicular_vector = perpendicular_vector.MakeSafeUnitVector(EPSILON);
          // Direction determined for the left-handed system.
          const cg::Vector3D scaled_perpendicular = perpendicular_vector * width;
          left_boundary.push_back(location + cg::Location(scaled_perpendicular));
          right_boundary.push_back(location + cg::Location(-1.0f * scaled_perpendicular));

          boundary_end = current_point;
        }

        current_point = waypoint_buffer.at(j);
      }

      // Reversing right boundary to construct clockwise (left-hand system)
      // boundary. This is so because both left and right boundary vectors have
      // the closest point to the vehicle at their starting index for the right
      // boundary,
      // we want to begin at the farthest point to have a clockwise trace.
      std::reverse(right_boundary.begin(), right_boundary.end());
      geodesic_boundary.insert(geodesic_boundary.end(), right_boundary.begin(), right_boundary.end());
      geodesic_boundary.insert(geodesic_boundary.end(), bbox.begin(), bbox.end());
      geodesic_boundary.insert(geodesic_boundary.end(), left_boundary.begin(), left_boundary.end());
    } else {

      geodesic_boundary = bbox;
    }

    geodesic_boundary_map.insert({actor.id, geodesic_boundary});
  }

  return geodesic_boundary;
}

Polygon FastCollisionStage::GetPolygon(const LocationVector &boundary) {

  traffic_manager::Polygon boundary_polygon;
  for (const cg::Location &location : boundary) {
    bg::append(boundary_polygon.outer(), Point2D(location.x, location.y));
  }
  bg::append(boundary_polygon.outer(), Point2D(boundary.front().x, boundary.front().y));

  return boundary_polygon;
}

GeometryComparison FastCollisionStage::GetGeometryBetweenActors(const ActorState & reference,
                                                                const ActorState & other) {
  uint64_t actor_id_key = std::min<uint64_t>(reference.id, other.id) | (std::max<uint64_t>(reference.id, other.id)<<32);

  GeometryComparison comparision_result{-1.0, -1.0, -1.0, -1.0};

  auto cache_i = geometry_cache.find(actor_id_key);
  if (cache_i != geometry_cache.end()) {
    comparision_result = cache_i->second;
    std::swap(comparision_result.reference_vehicle_to_other_geodesic,
              comparision_result.other_vehicle_to_reference_geodesic);
  } else {

    const Polygon reference_polygon = GetPolygon(GetBoundary(reference));
    const Polygon other_polygon = GetPolygon(GetBoundary(other));

    const Polygon reference_geodesic_polygon = GetPolygon(GetGeodesicBoundary(reference));

    const Polygon other_geodesic_polygon = GetPolygon(GetGeodesicBoundary(other));

    const float reference_vehicle_to_other_geodesic = static_cast<float>(bg::distance(reference_polygon, other_geodesic_polygon));
    const float other_vehicle_to_reference_geodesic = static_cast<float>(bg::distance(other_polygon, reference_geodesic_polygon));
    const float inter_geodesic_distance = static_cast<float>(bg::distance(reference_geodesic_polygon, other_geodesic_polygon));
    const float inter_bbox_distance = static_cast<float>(bg::distance(reference_polygon, other_polygon));

    comparision_result = {reference_vehicle_to_other_geodesic,
                          other_vehicle_to_reference_geodesic,
                          inter_geodesic_distance,
                          inter_bbox_distance};

    geometry_cache.insert({actor_id_key, comparision_result});
  }

  return comparision_result;
}

std::pair<bool, float> FastCollisionStage::NegotiateCollision(ActorState & reference,
                                                              const ActorState & other,
                                                              const uint64_t reference_junction_look_ahead_index) {
  // Output variables for the method.
  bool hazard = false;
  float available_distance_margin = std::numeric_limits<float>::infinity();

  const cg::Location reference_location = reference.kinematic_state.location;
  const cg::Location other_location = other.kinematic_state.location;

  // Ego and other vehicle heading.
  const cg::Vector3D reference_heading = reference.kinematic_state.rotation.GetForwardVector();
  // Vector from ego position to position of the other vehicle.
  cg::Vector3D reference_to_other = other_location - reference_location;
  reference_to_other = reference_to_other.MakeSafeUnitVector(EPSILON);

  // Other vehicle heading.
  const cg::Vector3D other_heading = other.kinematic_state.rotation.GetForwardVector();
  // Vector from other vehicle position to ego position.
  cg::Vector3D other_to_reference = reference_location - other_location;
  other_to_reference = other_to_reference.MakeSafeUnitVector(EPSILON);

  float reference_vehicle_length = GetDimensions(reference).x * SQUARE_ROOT_OF_TWO;
  float other_vehicle_length = GetDimensions(other).x * SQUARE_ROOT_OF_TWO;

  float inter_vehicle_distance = cg::Math::DistanceSquared(reference_location, other_location);
  float ego_bounding_box_extension = GetBoundingBoxExtention(reference);
  float other_bounding_box_extension = GetBoundingBoxExtention(other);
  // Calculate minimum distance between vehicle to consider collision negotiation.
  float inter_vehicle_length = reference_vehicle_length + other_vehicle_length;
  float ego_detection_range = SQUARE(ego_bounding_box_extension + inter_vehicle_length);
  float cross_detection_range = SQUARE(ego_bounding_box_extension + inter_vehicle_length + other_bounding_box_extension);

  // Conditions to consider collision negotiation.
  bool other_vehicle_in_ego_range = inter_vehicle_distance < ego_detection_range;
  bool other_vehicles_in_cross_detection_range = inter_vehicle_distance < cross_detection_range;
  float reference_heading_to_other_dot = cg::Math::Dot(reference_heading, reference_to_other);
  bool other_vehicle_in_front = reference_heading_to_other_dot > 0;
  const Buffer &reference_vehicle_buffer = reference.buffer;
  SimpleWaypointPtr closest_point = reference_vehicle_buffer.front();
  bool ego_inside_junction = closest_point->CheckJunction();
  TrafficLightState reference_tl_state = reference.traffic_light_state;
  bool ego_at_traffic_light = reference_tl_state.at_traffic_light;
  bool ego_stopped_by_light = reference_tl_state.tl_state != TLS::Green && reference_tl_state.tl_state != TLS::Off;
  SimpleWaypointPtr look_ahead_point = reference_vehicle_buffer.at(reference_junction_look_ahead_index);
  bool ego_at_junction_entrance = !closest_point->CheckJunction() && look_ahead_point->CheckJunction();

  // Conditions to consider collision negotiation.
  if (!(ego_at_junction_entrance && ego_at_traffic_light && ego_stopped_by_light)
      && ((ego_inside_junction && other_vehicles_in_cross_detection_range)
          || (!ego_inside_junction && other_vehicle_in_front && other_vehicle_in_ego_range))) {
    GeometryComparison geometry_comparison = GetGeometryBetweenActors(reference, other);

    // Conditions for collision negotiation.
    bool geodesic_path_bbox_touching = geometry_comparison.inter_geodesic_distance < 0.1;
    bool vehicle_bbox_touching = geometry_comparison.inter_bbox_distance < 0.1;
    bool ego_path_clear = geometry_comparison.other_vehicle_to_reference_geodesic > 0.1;
    bool other_path_clear = geometry_comparison.reference_vehicle_to_other_geodesic > 0.1;
    bool ego_path_priority = geometry_comparison.reference_vehicle_to_other_geodesic < geometry_comparison.other_vehicle_to_reference_geodesic;
    bool other_path_priority = geometry_comparison.reference_vehicle_to_other_geodesic > geometry_comparison.other_vehicle_to_reference_geodesic;
    bool ego_angular_priority = reference_heading_to_other_dot< cg::Math::Dot(other_heading, other_to_reference);

    // Whichever vehicle's path is farthest away from the other vehicle gets priority to move.
    bool lower_priority = !ego_path_priority && (other_path_priority || !ego_angular_priority);
    bool blocked_by_other_or_lower_priority = !ego_path_clear || (other_path_clear && lower_priority);
    bool yield_pre_crash = !vehicle_bbox_touching && blocked_by_other_or_lower_priority;
    bool yield_post_crash = vehicle_bbox_touching && !ego_angular_priority;

    if (geodesic_path_bbox_touching && (yield_pre_crash || yield_post_crash)) {

      hazard = true;

      const float reference_lead_distance = 2.f; // TODO: parameters.GetDistanceToLeadingVehicle(reference_vehicle_id);
      const float specific_distance_margin = std::max(reference_lead_distance, BOUNDARY_EXTENSION_MINIMUM);
      available_distance_margin = static_cast<float>(std::max(geometry_comparison.reference_vehicle_to_other_geodesic
                                                              - static_cast<double>(specific_distance_margin), 0.0));

      ///////////////////////////////////// Collision locking mechanism /////////////////////////////////
      // The idea is, when encountering a possible collision,
      // we should ensure that the bounding box extension doesn't decrease too fast and loose collision tracking.
      // This enables us to smoothly approach the lead vehicle.

      // When possible collision found, check if an entry for collision lock present.
      if (reference.collision_lock.lead_vehicle_id > 0) {
        auto &lock = reference.collision_lock;
        // Check if the same vehicle is under lock.
        if (other.id == lock.lead_vehicle_id) {
          // If the body of the lead vehicle is touching the reference vehicle bounding box.
          if (geometry_comparison.other_vehicle_to_reference_geodesic < 0.1) {
            // Distance between the bodies of the vehicles.
            lock.distance_to_lead_vehicle = geometry_comparison.inter_bbox_distance;
          } else {
            // Distance from reference vehicle body to other vehicle path polygon.
            lock.distance_to_lead_vehicle = geometry_comparison.reference_vehicle_to_other_geodesic;
          }
        } else {
          // If possible collision with a new vehicle, re-initialize with new lock entry.
          lock = {geometry_comparison.inter_bbox_distance, geometry_comparison.inter_bbox_distance, other.id};
        }
      } else {
        // Insert and initialize lock entry if not present.
        reference.collision_lock = {geometry_comparison.inter_bbox_distance,
                                    geometry_comparison.inter_bbox_distance,
                                    other.id};
      }
    }
  }

  // If no collision hazard detected, then flush collision lock held by the vehicle.
  if (!hazard) {
    reference.collision_lock = {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), 0u};
  }

  return {hazard, available_distance_margin};
}


} // namespace traffic_manager
} // namespace carla
