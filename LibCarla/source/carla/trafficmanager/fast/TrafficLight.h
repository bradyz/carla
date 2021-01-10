
#pragma once

#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/fast/ALSM.h"

namespace carla {
namespace traffic_manager {

struct GlobalParameters;
struct ActorParameters;

/// This class has functionality for responding to traffic lights
/// and managing entry into non-signalized junctions.
class FastTrafficLightStage {
private:
  FastALSM & alsm;
  const cc::World &world;
  /// Map containing the time ticket issued for vehicles.
  std::unordered_map<ActorId, cc::Timestamp> vehicle_last_ticket;
  /// Map containing the previous time ticket issued for junctions.
  std::unordered_map<JunctionID, cc::Timestamp> junction_last_ticket;
  /// Map containing the previous junction visited by a vehicle.
  std::unordered_map<ActorId, JunctionID> vehicle_last_junction;

  bool HandleNonSignalisedJunction(const ActorState & actor, const JunctionID junction_id,
                                   cc::Timestamp timestamp);

public:
  FastTrafficLightStage(FastALSM & alsm, const cc::World &world);

  void Update(ActorId actor_id,
              const ActorParameters &parameters,
              const GlobalParameters &global_parameters);
};

} // namespace traffic_manager
} // namespace carla
