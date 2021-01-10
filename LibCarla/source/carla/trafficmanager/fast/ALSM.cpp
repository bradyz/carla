#include "boost/pointer_cast.hpp"

#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"

#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/SimpleWaypoint.h"

#include "carla/trafficmanager/fast/ALSM.h"

namespace carla {
namespace traffic_manager {

FastALSM::FastALSM() {}

void FastALSM::Update(cc::World & world) {
  auto snapshot = world.GetSnapshot();
  double elapsed_seconds = snapshot.GetTimestamp().elapsed_seconds;
  auto world_actors = world.GetActors();

  // Tag all current actors for removal
  for(auto & i: actor_info)
    i.second.alive = false;

  // Handle the max idle
  double max_idle_time = elapsed_seconds;
  ActorId max_idle_actor = 0;

  // Update all world actors
  for (auto actor_ptr: *world_actors) {
    auto i = actor_info.find(actor_ptr->GetId());
    // Create a new actor if it doesn't exist
    if (i == actor_info.end())
      i = actor_info.insert({actor_ptr->GetId(), ActorState{actor_ptr->GetId()}}).first;
    auto &info = i->second;
    info.alive = true;

    // Update the actor
    const cg::Transform actor_transform = actor_ptr->GetTransform();
    const cg::Location actor_location = actor_transform.location;
    const cg::Rotation actor_rotation = actor_transform.rotation;
    const cg::Vector3D actor_velocity = actor_ptr->GetVelocity();
    float speed_limit = -1.0f;

    // I'm a bit allergic to static point casts based on a single character match..., things can go soo wrong
    auto vehicle_ptr = boost::dynamic_pointer_cast<cc::Vehicle>(actor_ptr);
    ActorType actor_type = ActorType::Pedestrian;

    // Update kinematics.
    if (vehicle_ptr) {
      actor_type = ActorType::Vehicle;
      speed_limit = vehicle_ptr->GetSpeedLimit();

      // Update the traffic light state for vehicles
      info.traffic_light_state = {vehicle_ptr->GetTrafficLightState(), vehicle_ptr->IsAtTrafficLight()};
    }

    // Update the attributes exactly once
    if (!info.static_attributes_set) {
      auto dimensions = actor_ptr->GetBoundingBox().extent;
      info.static_attributes_set = true;
      info.static_attributes = StaticAttributes{actor_type, dimensions.x, dimensions.y, dimensions.z};
      info.idle_time = elapsed_seconds;
    }

    // Update the kinematic state
    info.kinematic_state = KinematicState{actor_location, actor_rotation, actor_velocity, speed_limit, true};

    // Update the registered vehicle
    if (info.registered && vehicle_ptr) {
       //This generate a RPC call and is SLOW
       //vehicle_ptr->SetSimulatePhysics(true);

      // Updating idle time when necessary.
      if (actor_velocity.SquaredLength() > SQUARE(STOPPED_VELOCITY_THRESHOLD)) {
        info.idle_time = elapsed_seconds;
      }

      // Checking maximum idle time.
      if (max_idle_time > info.idle_time && info.IsStuck(elapsed_seconds)) {
        max_idle_time = info.idle_time;
        max_idle_actor = actor_ptr->GetId();
      }
    }

    // Update grid positions

    // TODO: track_traffic.UpdateUnregisteredGridPosition(actor_id, nearest_waypoints);
  }

  // Destroy registered vehicle if stuck at a location for too long.
  if (max_idle_time < elapsed_seconds
      && (elapsed_seconds - elapsed_last_actor_destruction) > DELTA_TIME_BETWEEN_DESTRUCTIONS) {
    actor_info[max_idle_actor].remove = true;
  }

  // Destroy all actors that were flagged
  for (auto actor_ptr: *world_actors) {
    auto i = actor_info.find(actor_ptr->GetId());
    if (i != actor_info.end() && i->second.remove) {
      actor_ptr->Destroy();
      i->second.alive = false;
    }
  }

  // Clean up the actor_info
  for(auto i = actor_info.begin(); i != actor_info.end();) {
    if (!i->second.alive)
      i = actor_info.erase(i);
    else
      ++i;
  }
}

bool ActorState::IsStuck(double elapsed_seconds) const {
  double delta_idle_time = elapsed_seconds - idle_time;
  return (!traffic_light_state.at_traffic_light
          && traffic_light_state.tl_state != TLS::Red
          && delta_idle_time >= BLOCKED_TIME_THRESHOLD)
         || (delta_idle_time >= RED_TL_BLOCKED_TIME_THRESHOLD);
}

void FastALSM::Reset() {
  actor_info.clear();
  elapsed_last_actor_destruction = 0.0;
}

//void FastALSM::RequestRemove(const ActorId actor_id) const {
//  auto i = actor_info.find(actor_id);
//  if (i != actor_info.end())
//    i->second.remove = true;
//}

const ActorState & FastALSM::GetState(const ActorId actor_id) const {
  auto i = actor_info.find(actor_id);
  if (i != actor_info.end())
    return i->second;
  static const ActorState no_info{0u, false, false};
  return no_info;
}
ActorState & FastALSM::GetState(const ActorId actor_id) {
  auto i = actor_info.find(actor_id);
  if (i != actor_info.end())
    return i->second;
  static ActorState no_info{0u, false, false};
  return no_info;
}

void FastALSM::Register(const ActorId actor_id) {
  auto i = actor_info.find(actor_id);
  if (i != actor_info.end())
    i->second.registered = true;
  else
    actor_info[actor_id] = {actor_id, false, true};
}
void FastALSM::Unregister(const ActorId actor_id) {
  auto i = actor_info.find(actor_id);
  if (i != actor_info.end())
    i->second.registered = false;
}


} // namespace traffic_manager
} // namespace carla
