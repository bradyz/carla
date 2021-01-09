
#pragma once

#include <memory>

#include "carla/client/World.h"
#include "carla/Memory.h"

//#include "carla/trafficmanager/AtomicActorSet.h"
//#include "carla/trafficmanager/CollisionStage.h"
#include "carla/trafficmanager/DataStructures.h"
//#include "carla/trafficmanager/InMemoryMap.h"
//#include "carla/trafficmanager/LocalizationStage.h"
//#include "carla/trafficmanager/MotionPlanStage.h"
#include "carla/trafficmanager/SimulationState.h"
#include "carla/trafficmanager/TrafficLightStage.h"

namespace carla {
namespace traffic_manager {

using namespace constants::HybridMode;
using namespace constants::VehicleRemoval;

namespace chr = std::chrono;
namespace cg = carla::geom;
namespace cc = carla::client;

class FastALSM {
public:
  struct Info {
    bool alive = true;
    bool registered = false;
    double idle_time = 0.f;
    bool remove = false;
    bool static_attributes_set = false;
    KinematicState kinematic_state;
    TrafficLightState traffic_light_state;
    StaticAttributes static_attributes;
    bool IsStuck(double elapsed_seconds) const;
  };

private:
  std::unordered_map<ActorId, Info> actor_info;

  // Time elapsed since last vehicle destruction due to being idle for too long.
  double elapsed_last_actor_destruction {0.0};

public:
  FastALSM();

  void Update(cc::World & world);

  // Removes an actor from traffic manager and performs clean up of associated data
  void RequestRemove(const ActorId actor_id) const;

  // from various stages tracking the said vehicle.
  const Info & ActorInfo(const ActorId actor_id) const;

  void Register(const ActorId actor_id);
  void Unregister(const ActorId actor_id);

  void Reset();
};

} // namespace traffic_manager
} // namespace carla
