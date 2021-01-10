#pragma once

#include "carla/trafficmanager/fast/ALSM.h"
#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/TrackTraffic.h"

namespace carla {
namespace traffic_manager {

namespace cc = carla::client;

using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using LaneChangeLocationMap = std::unordered_map<ActorId, cg::Location>;

struct GlobalParameters;
struct ActorParameters;
struct ActorState;

class FastLocalizationStage {
protected:
  // Owned by localization, no other stage will modify this
  BufferMap buffer_map;
  TrackTraffic track_traffic;
  LaneChangeLocationMap last_lane_change_location;
  using SimpleWaypointPair = std::pair<SimpleWaypointPtr, SimpleWaypointPtr>;
  std::unordered_map <ActorId, SimpleWaypointPair> vehicles_at_junction_entrance;

  // Const members
  LocalMapPtr local_map;


  SimpleWaypointPtr AssignLaneChange(const ActorId actor_id,
                                     const cg::Location vehicle_location,
                                     const float vehicle_speed,
                                     bool force, bool direction);

  void ExtendAndFindSafeSpace(const ActorId actor_id,
                              const bool is_at_junction_entrance,
                              Buffer &waypoint_buffer);

public:
  FastLocalizationStage(LocalMapPtr local_map);

  void Update(ActorId actor_id,
              ActorState &state,
              const ActorParameters &parameters,
              const GlobalParameters &global_parameters);
};

} // namespace traffic_manager
} // namespace carla
