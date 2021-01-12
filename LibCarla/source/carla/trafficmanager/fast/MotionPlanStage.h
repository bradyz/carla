
/// This file has functionality for motion planning based on information
/// from localization, collision avoidance and traffic light response.

#pragma once

#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/LocalizationUtils.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/SimulationState.h"
#include "carla/trafficmanager/TrackTraffic.h"
#include "carla/trafficmanager/fast/ALSM.h"

namespace carla {
namespace traffic_manager {

struct GlobalParameters;
struct ActorParameters;

class FastMotionPlanStage {
private:
  FastALSM & alsm;

  const TrackTraffic &track_traffic;
  // PID paramenters for various road conditions.
  const std::vector<float> urban_longitudinal_parameters;
  const std::vector<float> highway_longitudinal_parameters;
  const std::vector<float> urban_lateral_parameters;
  const std::vector<float> highway_lateral_parameters;
  const cc::World &world;
  // Structure holding the controller state for registered vehicles.
  std::unordered_map<ActorId, StateEntry> pid_state_map;
  // Structure to keep track of duration between teleportation
  // in hybrid physics mode.
  std::unordered_map<ActorId, cc::Timestamp> teleportation_instance;
  cc::Timestamp current_timestamp;

  std::pair<bool, float> CollisionHandling(const CollisionHazardData &collision_hazard,
                                           const bool tl_hazard,
                                           const cg::Vector3D ego_velocity,
                                           const cg::Vector3D ego_heading,
                                           const float max_target_velocity);

bool SafeAfterJunction(const LocalizationData &localization,
                       const bool tl_hazard,
                       const bool collision_emergency_stop);

public:
  FastMotionPlanStage(FastALSM & alsm,
                      const TrackTraffic &track_traffic,
                      const std::vector<float> &urban_longitudinal_parameters,
                      const std::vector<float> &highway_longitudinal_parameters,
                      const std::vector<float> &urban_lateral_parameters,
                      const std::vector<float> &highway_lateral_parameters,
                      const cc::World &world);

  void Update(ActorId actor_id,
              const ActorParameters &parameters,
              const GlobalParameters &global_parameters);
};

} // namespace traffic_manager
} // namespace carla
