
#pragma once

#include <memory>

#include "carla/client/World.h"
#include "carla/Memory.h"

//#include "carla/trafficmanager/AtomicActorSet.h"
//#include "carla/trafficmanager/CollisionStage.h"
#include "carla/trafficmanager/Constants.h"
#include "carla/trafficmanager/DataStructures.h"
//#include "carla/trafficmanager/InMemoryMap.h"
//#include "carla/trafficmanager/LocalizationStage.h"
//#include "carla/trafficmanager/MotionPlanStage.h"
#include "carla/trafficmanager/RandomGenerator.h"
#include "carla/trafficmanager/SimulationState.h"
//#include "carla/trafficmanager/TrafficLightStage.h"

namespace carla {
namespace traffic_manager {

using namespace constants::HybridMode;
using namespace constants::VehicleRemoval;

namespace chr = std::chrono;
namespace cg = carla::geom;
namespace cc = carla::client;


struct ActorState {
  struct CollisionLock {
    float distance_to_lead_vehicle = std::numeric_limits<float>::infinity();
    float initial_lock_distance = std::numeric_limits<float>::infinity();
    ActorId lead_vehicle_id = 0u;
  };

  // ALSM State
  ActorId id;
  bool alive = true;
  bool registered = false;
  double idle_time = 0.f;
  bool remove = false;
  bool static_attributes_set = false;
  KinematicState kinematic_state = {cg::Location(), cg::Rotation(), cg::Vector3D(), 0.f, true};
  TrafficLightState traffic_light_state = {TLS::Unknown, false};
  StaticAttributes static_attributes = {ActorType::Any, 0.f, 0.f, 0.f};
  RandomGenerator random_device = RandomGenerator(0);
  bool IsStuck(double elapsed_seconds) const;

  // Localization State
  LocalizationData localization =  {nullptr, nullptr, false};
  Buffer buffer = {};

  // Collision state
  CollisionHazardData collision = {std::numeric_limits<float>::infinity(), 0, false};
  CollisionLock collision_lock = {};

  // TrafficLight state
  bool traffic_light_hazard = false;
};

class FastALSM {
private:
  std::unordered_map<ActorId, ActorState> actor_info;

  // Time elapsed since last vehicle destruction due to being idle for too long.
  double elapsed_last_actor_destruction {0.0};

public:
  FastALSM();

  void Update(cc::World & world);
//
//  // Removes an actor from traffic manager and performs clean up of associated data
//  void RequestRemove(const ActorId actor_id) const;

  // from various stages tracking the said vehicle.
  const ActorState & GetState(const ActorId actor_id) const;
  ActorState & GetState(const ActorId actor_id);

  void Register(const ActorId actor_id);
  void Unregister(const ActorId actor_id);

  void Reset();
};

} // namespace traffic_manager
} // namespace carla
