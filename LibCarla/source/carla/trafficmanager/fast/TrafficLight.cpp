
#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/LocalizationUtils.h"

#include "carla/trafficmanager/fast/TrafficLight.h"
#include "carla/trafficmanager/fast/TrafficManagerLocal.h"

namespace carla {
namespace traffic_manager {

using constants::TrafficLight::DOUBLE_NO_SIGNAL_PASSTHROUGH_INTERVAL;
using constants::WaypointSelection::JUNCTION_LOOK_AHEAD;

FastTrafficLightStage::FastTrafficLightStage(FastALSM & alsm, const cc::World &world)
        : alsm(alsm),
          world(world) {}

void FastTrafficLightStage::Update(ActorId ego_actor_id,
            const ActorParameters &parameters,
            const GlobalParameters &global_parameters) {
  bool traffic_light_hazard = false;

  auto & state = alsm.GetState(ego_actor_id);

  const Buffer &waypoint_buffer = state.buffer;
  const SimpleWaypointPtr look_ahead_point = GetTargetWaypoint(waypoint_buffer, JUNCTION_LOOK_AHEAD).first;

  const JunctionID junction_id = look_ahead_point->GetWaypoint()->GetJunctionId();
  cc::Timestamp current_timestamp = world.GetSnapshot().GetTimestamp();

  const TrafficLightState tl_state = state.traffic_light_state;
  const TLS traffic_light_state = tl_state.tl_state;
  const bool is_at_traffic_light = tl_state.at_traffic_light;

  // We determine to stop if the current position of the vehicle is not a
  // junction and there is a red or yellow light.
  if (is_at_traffic_light &&
      traffic_light_state != TLS::Green &&
      traffic_light_state != TLS::Off &&
      parameters.percentage_running_light <= state.random_device.next()) {

    traffic_light_hazard = true;
  }
    // Handle entry negotiation at non-signalised junction.
  else if (look_ahead_point->CheckJunction() &&
           !is_at_traffic_light &&
           traffic_light_state != TLS::Green &&
           traffic_light_state != TLS::Off &&
           parameters.percentage_running_sign <= state.random_device.next()) {

    traffic_light_hazard = HandleNonSignalisedJunction(state, junction_id, current_timestamp);
  }

  state.traffic_light_hazard = traffic_light_hazard;
}

bool FastTrafficLightStage::HandleNonSignalisedJunction(const ActorState & actor, const JunctionID junction_id,
                                                    cc::Timestamp timestamp) {

  bool traffic_light_hazard = false;
  auto i = vehicle_last_junction.find(actor.id);
  if (i == vehicle_last_junction.end() || i->second != junction_id) {
    vehicle_last_junction[actor.id] = junction_id;

    // Check if the vehicle has an outdated ticket or needs a new one.
    bool need_to_issue_new_ticket = false;
    auto previous_ticket = vehicle_last_ticket.find(actor.id);
    if (previous_ticket == vehicle_last_ticket.end()) {
      need_to_issue_new_ticket = true;
    } else {
      const double diff = timestamp.elapsed_seconds - previous_ticket->second.elapsed_seconds;
      if (diff > DOUBLE_NO_SIGNAL_PASSTHROUGH_INTERVAL) {
        need_to_issue_new_ticket = true;
      }
    }

    // If new ticket is needed for the vehicle, then query the junction
    // ticket map
    // and update the map value to the new ticket value.
    if (need_to_issue_new_ticket) {
      if (junction_last_ticket.find(junction_id) != junction_last_ticket.end()) {

        cc::Timestamp &last_ticket = junction_last_ticket.at(junction_id);
        const double diff = timestamp.elapsed_seconds - last_ticket.elapsed_seconds;
        if (diff > 0.0) {
          last_ticket.elapsed_seconds = timestamp.elapsed_seconds + DOUBLE_NO_SIGNAL_PASSTHROUGH_INTERVAL;
        } else {
          last_ticket.elapsed_seconds += DOUBLE_NO_SIGNAL_PASSTHROUGH_INTERVAL;
        }
      } else {
        cc::Timestamp &new_ticket = timestamp;
        new_ticket.elapsed_seconds += DOUBLE_NO_SIGNAL_PASSTHROUGH_INTERVAL;
        junction_last_ticket.insert({junction_id, new_ticket});
      }
      if (vehicle_last_ticket.find(actor.id) != vehicle_last_ticket.end()) {
        vehicle_last_ticket.at(actor.id) = junction_last_ticket.at(junction_id);
      } else {
        vehicle_last_ticket.insert({actor.id, junction_last_ticket.at(junction_id)});
      }
    }
  }

  // If current time is behind ticket time, then do not enter junction.
  const cc::Timestamp current_ticket = vehicle_last_ticket.at(actor.id);
  const double diff = current_ticket.elapsed_seconds - timestamp.elapsed_seconds;
  if (diff > 0.0) {
    traffic_light_hazard = true;
  }

  return traffic_light_hazard;
}


} // namespace traffic_manager
} // namespace carla
