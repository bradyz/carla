// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "carla/client/DebugHelper.h"
#include "carla/client/detail/EpisodeProxy.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/World.h"
#include "carla/Memory.h"
#include "carla/rpc/Command.h"

#include "carla/trafficmanager/AtomicActorSet.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/TrackTraffic.h"
#include "carla/trafficmanager/TrafficManagerBase.h"
#include "carla/trafficmanager/TrafficManagerServer.h"

#include "carla/trafficmanager/fast/ALSM.h"

namespace carla {
namespace traffic_manager {

namespace chr = std::chrono;

using namespace std::chrono_literals;

using TimePoint = chr::time_point<chr::system_clock, chr::nanoseconds>;
using TLGroup = std::vector<carla::SharedPtr<carla::client::TrafficLight>>;
using LocalMapPtr = std::shared_ptr<InMemoryMap>;
using constants::HybridMode::HYBRID_MODE_DT;

struct GlobalParameters {
  bool synchronous_mode = false;
  bool hybrid_physics_mode = false;
  // Not sure if this should be atomic
  std::chrono::duration<double, std::milli> synchronous_timeout = 10ms;
  float percentage_speed_difference = 0;
  float distance_to_leading_vehicle = 2.f;
  float hybrid_physics_radius = 70.f;
  bool osm_mode = true;
  uint64_t seed {static_cast<uint64_t>(time(NULL))};
};

struct ActorParameters {
  float percentage_speed_difference = -1.f;
  ChangeLaneInfo force_lane_change = {false, false};
  bool auto_lane_change = true;
  float distance_to_leading_vehicle = -1.f;
  float percentage_ignore_walkers = 0.f;
  float percentage_ignore_vehicles = 0.f;
  float percentage_running_light = 0.f;
  float percentage_running_sign = 0.f;
  float percentage_keep_right = -1.f;
  std::unordered_set<ActorId> cannot_collide;
  bool CanCollide(ActorId other) const {
    return !cannot_collide.count(other);
  }

  // Returns an ActorParameters with default values for unset variables
  // and resets force_lane_change and and keep_right of this
  ActorParameters update_and_reset(const GlobalParameters &global);
};

/// The function of this class is to integrate all the various stages of
/// the traffic manager appropriately using messengers.
class FastTrafficManagerLocal : public TrafficManagerBase {
protected:
  /// PID controller parameters.
  std::vector<float> longitudinal_PID_parameters;
  std::vector<float> longitudinal_highway_PID_parameters;
  std::vector<float> lateral_PID_parameters;
  std::vector<float> lateral_highway_PID_parameters;
  /// Carla's client connection object.
  carla::client::detail::EpisodeProxy episode_proxy;

  GlobalParameters global_parameters;

  std::mutex actor_parameters_mutex;
  std::unordered_map<ActorId, std::shared_ptr<ActorParameters>> actor_parameters;
  unsigned long num_actors = 0u;

  std::shared_ptr<ActorParameters> GetActorParameters(const ActorPtr &actor);


public:
  /// To start the traffic manager.
  virtual void Start();

  /// To stop the traffic manager.
  virtual void Stop();

  /// Initiates thread to run the TrafficManager sequentially.
  void Run();

  /// To release the traffic manager.
  virtual void Release();

  /// To reset the traffic manager.
  virtual void Reset();

  /// Protected constructor for singleton lifecycle management.
  FastTrafficManagerLocal(std::vector<float> longitudinal_PID_parameters,
                          std::vector<float> longitudinal_highway_PID_parameters,
                          std::vector<float> lateral_PID_parameters,
                          std::vector<float> lateral_highway_PID_parameters,
                          float perc_decrease_from_limit,
                          cc::detail::EpisodeProxy episode_proxy,
                          uint16_t RPCportTM);

  virtual ~FastTrafficManagerLocal();

  /// This method registers a vehicle with the traffic manager.
  virtual void RegisterVehicles(const std::vector <ActorPtr> &actor_list);

  /// This method unregisters a vehicle from traffic manager.
  virtual void UnregisterVehicles(const std::vector <ActorPtr> &actor_list);

  /// Set a vehicle's % decrease in velocity with respect to the speed limit.
  /// If less than 0, it's a % increase.
  virtual void SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage);

  /// Set a global % decrease in velocity with respect to the speed limit.
  /// If less than 0, it's a % increase.
  virtual void SetGlobalPercentageSpeedDifference(float const percentage);

  /// Method to set collision detection rules between vehicles.
  virtual void
  SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision);

  /// Method to force lane change on a vehicle.
  /// Direction flag can be set to true for left and false for right.
  virtual void SetForceLaneChange(const ActorPtr &actor, const bool direction);

  /// Enable/disable automatic lane change on a vehicle.
  virtual void SetAutoLaneChange(const ActorPtr &actor, const bool enable);

  /// Method to specify how much distance a vehicle should maintain to
  /// the leading vehicle.
  virtual void SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance);

  /// Method to specify the % chance of ignoring collisions with any walker.
  virtual void SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc);

  /// Method to specify the % chance of ignoring collisions with any vehicle.
  virtual void SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc);

  /// Method to specify the % chance of running any traffic light.
  virtual void SetPercentageRunningLight(const ActorPtr &actor, const float perc);

  /// Method to specify the % chance of running any traffic sign.
  virtual void SetPercentageRunningSign(const ActorPtr &actor, const float perc);

  /// Method to switch traffic manager into synchronous execution.
  virtual void SetSynchronousMode(bool mode);

  /// Method to set Tick timeout for synchronous execution.
  virtual void SetSynchronousModeTimeOutInMiliSecond(double time);

  /// Method to provide synchronous tick
  virtual bool SynchronousTick();

  /// Get carla episode information
  virtual carla::client::detail::EpisodeProxy &GetEpisodeProxy() {
    return episode_proxy;
  }

  /// Method to set Global Distance to Leading Vehicle.
  virtual void SetGlobalDistanceToLeadingVehicle(const float dist);

  /// Method to set probabilistic preference to keep on the right lane.
  virtual void SetKeepRightPercentage(const ActorPtr &actor, const float percentage);

  /// Method to set hybrid physics mode.
  virtual void SetHybridPhysicsMode(const bool mode_switch);

  /// Method to set hybrid physics radius.
  virtual void SetHybridPhysicsRadius(const float radius);

  /// Method to set randomization seed.
  virtual void SetRandomDeviceSeed(const uint64_t seed);

  /// Method to set Open Street Map mode.
  virtual void SetOSMMode(const bool mode_switch);

protected:
  /// Traffic manager server instance.
  TrafficManagerServer server;

  /// Switch to turn on / turn off traffic manager.
  std::atomic<bool> run_traffic_manger{true};
  /// Flags to signal step begin and end.
  std::atomic<bool> step_begin{false};
  std::atomic<bool> step_end{false};
  /// Mutex for progressing synchronous execution.
  std::mutex step_execution_mutex;
  /// Condition variables for progressing synchronous execution.
  std::condition_variable step_begin_trigger;
  std::condition_variable step_end_trigger;
  /// Single worker thread for sequential execution of sub-components.
  std::unique_ptr <std::thread> worker_thread;
  /// Mutex to prevent vehicle registration during frame array re-allocation.
  std::mutex registration_mutex;

protected: // ALSM
  FastALSM alsm;

};

} // namespace traffic_manager
} // namespace carla
