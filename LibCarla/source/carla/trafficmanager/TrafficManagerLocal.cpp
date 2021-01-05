// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "carla/client/detail/Simulator.h"

#include "carla/trafficmanager/TrafficManagerLocal.h"
#include "carla/trafficmanager/SnippetProfiler.h"

namespace carla {
namespace traffic_manager {

using namespace constants::FrameMemory;

TrafficManagerLocal::TrafficManagerLocal(
  std::vector<float> longitudinal_PID_parameters,
  std::vector<float> longitudinal_highway_PID_parameters,
  std::vector<float> lateral_PID_parameters,
  std::vector<float> lateral_highway_PID_parameters,
  float perc_difference_from_limit,
  cc::detail::EpisodeProxy &episode_proxy,
  uint16_t &RPCportTM)

  : longitudinal_PID_parameters(longitudinal_PID_parameters),
    longitudinal_highway_PID_parameters(longitudinal_highway_PID_parameters),
    lateral_PID_parameters(lateral_PID_parameters),
    lateral_highway_PID_parameters(lateral_highway_PID_parameters),

    episode_proxy(episode_proxy),
    world(cc::World(episode_proxy)),
    debug_helper(world.MakeDebugHelper()),

    localization_stage(LocalizationStage(vehicle_id_list,
                                         buffer_map,
                                         simulation_state,
                                         track_traffic,
                                         local_map,
                                         parameters,
                                         marked_for_removal,
                                         localization_frame,
                                         debug_helper,
                                         random_devices)),

    collision_stage(CollisionStage(vehicle_id_list,
                                   simulation_state,
                                   buffer_map,
                                   track_traffic,
                                   parameters,
                                   collision_frame,
                                   debug_helper,
                                   random_devices)),

    traffic_light_stage(TrafficLightStage(vehicle_id_list,
                                          simulation_state,
                                          buffer_map,
                                          parameters,
                                          world,
                                          tl_frame,
                                          random_devices)),

    motion_plan_stage(MotionPlanStage(vehicle_id_list,
                                      simulation_state,
                                      parameters,
                                      buffer_map,
                                      track_traffic,
                                      longitudinal_PID_parameters,
                                      longitudinal_highway_PID_parameters,
                                      lateral_PID_parameters,
                                      lateral_highway_PID_parameters,
                                      localization_frame,
                                      collision_frame,
                                      tl_frame,
                                      world,
                                      control_frame)),

    alsm(ALSM(registered_vehicles,
              buffer_map,
              track_traffic,
              marked_for_removal,
              parameters,
              world,
              local_map,
              simulation_state,
              localization_stage,
              collision_stage,
              traffic_light_stage,
              motion_plan_stage,
              random_devices)),

    server(TrafficManagerServer(RPCportTM, static_cast<carla::traffic_manager::TrafficManagerBase *>(this))) {

  parameters.SetGlobalPercentageSpeedDifference(perc_difference_from_limit);

  registered_vehicles_state = -1;

  SetupLocalMap();

  Start();
}

TrafficManagerLocal::~TrafficManagerLocal() {
  episode_proxy.Lock()->DestroyTrafficManager(server.port());
  Release();
}

void TrafficManagerLocal::SetupLocalMap() {
  const carla::SharedPtr<cc::Map> world_map = world.GetMap();
  local_map = std::make_shared<InMemoryMap>(world_map);
  local_map->SetUp();
}

void TrafficManagerLocal::Start() {
  run_traffic_manger.store(true);
  worker_thread = std::make_unique<std::thread>(&TrafficManagerLocal::Run, this);
}

#if 1
#define TIMER(x) static TicToc timer(x)
#define TIC timer.tic()
#define TOC(x) timer.toc(x)
#define FINISH timer.finish()
#else
#define TIMER(x)
#define TIC
#define TOC(x)
#define FINISH
#endif

void TrafficManagerLocal::Run() {

  localization_frame.reserve(INITIAL_SIZE);
  collision_frame.reserve(INITIAL_SIZE);
  tl_frame.reserve(INITIAL_SIZE);
  control_frame.reserve(INITIAL_SIZE);
  current_reserved_capacity = INITIAL_SIZE;

  TIMER("TrafficManager");
  while (run_traffic_manger.load()) {
    TIC;
    
    bool synchronous_mode = parameters.GetSynchronousMode();
    bool hybrid_physics_mode = parameters.GetHybridPhysicsMode();
    TOC("param");

    // Wait for external trigger to initiate cycle in synchronous mode.
    if (synchronous_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      step_begin_trigger.wait(lock, [this]() {return step_begin.load() || !run_traffic_manger.load();});
      step_begin.store(false);
    }
    TOC("sync");

    // Skipping velocity update if elapsed time is less than 0.05s in asynchronous, hybrid mode.
    if (!synchronous_mode && hybrid_physics_mode) {
      TimePoint current_instance = chr::system_clock::now();
      chr::duration<float> elapsed_time = current_instance - previous_update_instance;
      chr::duration<float> time_to_wait = chr::duration<float>(HYBRID_MODE_DT) - elapsed_time;
      if (time_to_wait > chr::duration<float>(0.0f)) {
        std::this_thread::sleep_for(time_to_wait);
      }
      previous_update_instance = current_instance;
    }
    TOC("hybrid");

    std::unique_lock<std::mutex> registration_lock(registration_mutex);
    TOC("registration_lock");
    
    // Updating simulation state, actor life cycle and performing necessary cleanup.
    alsm.Update();
    TOC("alsm.Update");

    // Re-allocating inter-stage communication frames based on changed number of registered vehicles.
    int current_registered_vehicles_state = registered_vehicles.GetState();
    unsigned long number_of_vehicles = vehicle_id_list.size();
    if (registered_vehicles_state != current_registered_vehicles_state || number_of_vehicles != registered_vehicles.Size()) {

      vehicle_id_list = registered_vehicles.GetIDList();

      std::sort(vehicle_id_list.begin(), vehicle_id_list.end());

      number_of_vehicles = vehicle_id_list.size();

      // Reserve more space if needed.
      uint64_t growth_factor = static_cast<uint64_t>(static_cast<float>(number_of_vehicles) * INV_GROWTH_STEP_SIZE);
      uint64_t new_frame_capacity = INITIAL_SIZE + GROWTH_STEP_SIZE * growth_factor;
      if (new_frame_capacity > current_reserved_capacity) {
        localization_frame.reserve(new_frame_capacity);
        collision_frame.reserve(new_frame_capacity);
        tl_frame.reserve(new_frame_capacity);
        control_frame.reserve(new_frame_capacity);
      }

      registered_vehicles_state = registered_vehicles.GetState();
    }
    TOC("inter-stage");

    // Reset frames for current cycle.
    localization_frame.clear();
    localization_frame.resize(number_of_vehicles);
    collision_frame.clear();
    collision_frame.resize(number_of_vehicles);
    tl_frame.clear();
    tl_frame.resize(number_of_vehicles);
    control_frame.clear();
    control_frame.resize(number_of_vehicles);
    TOC("reset");

    // Run core operation stages.
//     for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
//       localization_stage.Update(index);
//     }
//     TOC("localization_stage");
//     for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
//       collision_stage.Update(index);
//     }
//     collision_stage.ClearCycleCache();
//     TOC("collision_stage");
// 
//     for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
//       traffic_light_stage.Update(index);
//       motion_plan_stage.Update(index);
//     }
//     TOC("traffic_light_stage");

    registration_lock.unlock();
    TOC("registration_lock.unlock");

    // Sending the current cycle's batch command to the simulator.
    if (synchronous_mode) {
      // If TCP and RPC preserve the call order then ApplyBatchSync is overkill here.
      // We do not need the response and only require the control_frame to be executed
      // before any other client commands
      episode_proxy.Lock()->ApplyBatch(control_frame, false);
      step_end.store(true);
      step_end_trigger.notify_one();
    } else {
      episode_proxy.Lock()->ApplyBatch(control_frame, false);
    }
    TOC("end");
    FINISH;
  }
}

bool TrafficManagerLocal::SynchronousTick() {
  if (parameters.GetSynchronousMode()) {
    step_begin.store(true);
    step_begin_trigger.notify_one();

    std::unique_lock<std::mutex> lock(step_execution_mutex);
    step_end_trigger.wait(lock, [this]() { return step_end.load(); });
    step_end.store(false);
  }
  return true;
}

void TrafficManagerLocal::Stop() {

  run_traffic_manger.store(false);
  if (parameters.GetSynchronousMode()) {
    step_begin_trigger.notify_one();
  }

  if (worker_thread) {
    if (worker_thread->joinable()) {
      worker_thread->join();
    }
    worker_thread.release();
  }

  vehicle_id_list.clear();
  registered_vehicles.Clear();
  registered_vehicles_state = -1;
  track_traffic.Clear();
  previous_update_instance = chr::system_clock::now();
  current_reserved_capacity = 0u;
  random_devices.clear();

  simulation_state.Reset();
  localization_stage.Reset();
  collision_stage.Reset();
  traffic_light_stage.Reset();
  motion_plan_stage.Reset();

  buffer_map.clear();
  localization_frame.clear();
  collision_frame.clear();
  tl_frame.clear();
  control_frame.clear();

  run_traffic_manger.store(true);
  step_begin.store(false);
  step_end.store(false);
}

void TrafficManagerLocal::Release() {

  Stop();

  local_map.reset();
}

void TrafficManagerLocal::Reset() {

  Release();
  episode_proxy = episode_proxy.Lock()->GetCurrentEpisode();
  world = cc::World(episode_proxy);
  SetupLocalMap();
  Start();
}

void TrafficManagerLocal::RegisterVehicles(const std::vector<ActorPtr> &vehicle_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  registered_vehicles.Insert(vehicle_list);
  for (const ActorPtr &vehicle: vehicle_list) {
    random_devices.insert({vehicle->GetId(), RandomGenerator(seed)});
  }
}

void TrafficManagerLocal::UnregisterVehicles(const std::vector<ActorPtr> &actor_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  std::vector<ActorId> actor_id_list;
  for (auto &actor : actor_list) {
    alsm.RemoveActor(actor->GetId(), true);
  }
}

void TrafficManagerLocal::SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage) {
  parameters.SetPercentageSpeedDifference(actor, percentage);
}

void TrafficManagerLocal::SetGlobalPercentageSpeedDifference(const float percentage) {
  parameters.SetGlobalPercentageSpeedDifference(percentage);
}

void TrafficManagerLocal::SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision) {
  parameters.SetCollisionDetection(reference_actor, other_actor, detect_collision);
}

void TrafficManagerLocal::SetForceLaneChange(const ActorPtr &actor, const bool direction) {
  parameters.SetForceLaneChange(actor, direction);
}

void TrafficManagerLocal::SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
  parameters.SetAutoLaneChange(actor, enable);
}

void TrafficManagerLocal::SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
  parameters.SetDistanceToLeadingVehicle(actor, distance);
}

void TrafficManagerLocal::SetGlobalDistanceToLeadingVehicle(const float distance) {
  parameters.SetGlobalDistanceToLeadingVehicle(distance);
}

void TrafficManagerLocal::SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreWalkers(actor, perc);
}

void TrafficManagerLocal::SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreVehicles(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningLight(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningLight(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningSign(actor, perc);
}

void TrafficManagerLocal::SetKeepRightPercentage(const ActorPtr &actor, const float percentage) {
  parameters.SetKeepRightPercentage(actor, percentage);
}

void TrafficManagerLocal::SetHybridPhysicsMode(const bool mode_switch) {
  parameters.SetHybridPhysicsMode(mode_switch);
}

void TrafficManagerLocal::SetHybridPhysicsRadius(const float radius) {
  parameters.SetHybridPhysicsRadius(radius);
}

void TrafficManagerLocal::SetOSMMode(const bool mode_switch) {
  parameters.SetOSMMode(mode_switch);
}

bool TrafficManagerLocal::CheckAllFrozen(TLGroup tl_to_freeze) {
  for (auto &elem : tl_to_freeze) {
    if (!elem->IsFrozen() || elem->GetState() != TLS::Red) {
      return false;
    }
  }
  return true;
}

void TrafficManagerLocal::SetSynchronousMode(bool mode) {
  const bool previous_mode = parameters.GetSynchronousMode();
  parameters.SetSynchronousMode(mode);
  if (previous_mode && !mode) {
    step_begin.store(true);
    step_begin_trigger.notify_one();
  }
}

void TrafficManagerLocal::SetSynchronousModeTimeOutInMiliSecond(double time) {
  parameters.SetSynchronousModeTimeOutInMiliSecond(time);
}

carla::client::detail::EpisodeProxy &TrafficManagerLocal::GetEpisodeProxy() {
  return episode_proxy;
}

std::vector<ActorId> TrafficManagerLocal::GetRegisteredVehiclesIDs() {
  return registered_vehicles.GetIDList();
}

void TrafficManagerLocal::SetRandomDeviceSeed(const uint64_t _seed) {
  seed = _seed;
  world.ResetAllTrafficLights();
}



void FastTrafficManagerLocal::Start() {
  // Start the worker thread
  run_traffic_manger.store(true);
  worker_thread = std::make_unique<std::thread>(&FastTrafficManagerLocal::Run, this);
}

void FastTrafficManagerLocal::Stop() {
  // Stop the worker thread
  run_traffic_manger.store(false);
  if (global_parameters.synchronous_mode) {
    step_begin_trigger.notify_one();
  }

  if (worker_thread) {
    if (worker_thread->joinable()) {
      worker_thread->join();
    }
    worker_thread.release();
  }
}

void FastTrafficManagerLocal::Release() {
  Stop();
}

void FastTrafficManagerLocal::Reset() {
  Release();
  episode_proxy = episode_proxy.Lock()->GetCurrentEpisode();
  Start();
}

FastTrafficManagerLocal::FastTrafficManagerLocal(std::vector<float> longitudinal_PID_parameters,
                          std::vector<float> longitudinal_highway_PID_parameters,
                          std::vector<float> lateral_PID_parameters,
                          std::vector<float> lateral_highway_PID_parameters,
                          float perc_decrease_from_limit,
                          cc::detail::EpisodeProxy episode_proxy,
                          uint16_t RPCportTM)
                          
  : longitudinal_PID_parameters(longitudinal_PID_parameters),
    longitudinal_highway_PID_parameters(longitudinal_highway_PID_parameters),
    lateral_PID_parameters(lateral_PID_parameters),
    lateral_highway_PID_parameters(lateral_highway_PID_parameters),
    episode_proxy(episode_proxy),
    server(TrafficManagerServer(RPCportTM, static_cast<carla::traffic_manager::TrafficManagerBase *>(this)))  {
  global_parameters.percentage_speed_difference = perc_decrease_from_limit;
  Start();
}

FastTrafficManagerLocal::~FastTrafficManagerLocal() {
  episode_proxy.Lock()->DestroyTrafficManager(server.port());
  Release();
}

  /// This method registers a vehicle with the traffic manager.
void FastTrafficManagerLocal::RegisterVehicles(const std::vector<ActorPtr> &actor_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  for (const ActorPtr &vehicle: actor_list)
    alsm.Register(vehicle->GetId());
}

  /// This method unregisters a vehicle from traffic manager.
void FastTrafficManagerLocal::UnregisterVehicles(const std::vector<ActorPtr> &actor_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  for (const ActorPtr &vehicle: actor_list)
    alsm.Unregister(vehicle->GetId());
}

  /// Method to provide synchronous tick
bool FastTrafficManagerLocal::SynchronousTick() {
  if (global_parameters.synchronous_mode) {
    step_begin.store(true);
    step_begin_trigger.notify_one();

    std::unique_lock<std::mutex> lock(step_execution_mutex);
    step_end_trigger.wait(lock, [this]() { return step_end.load(); });
    step_end.store(false);
  }
  return true;
}


void FastTrafficManagerLocal::SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage)  {
  GetActorInfo(actor)->percentage_speed_difference = percentage;
}

void FastTrafficManagerLocal::SetGlobalPercentageSpeedDifference(float const percentage) {
  global_parameters.percentage_speed_difference = percentage;
}

/// Method to set collision detection rules between vehicles.
void FastTrafficManagerLocal::SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision) { /* TODO */ }

void FastTrafficManagerLocal::SetForceLaneChange(const ActorPtr &actor, const bool direction) {
  GetActorInfo(actor)->force_lane_change = {true, direction};
}

void FastTrafficManagerLocal::SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
  GetActorInfo(actor)->auto_lane_change = enable;
}

void FastTrafficManagerLocal::SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
  GetActorInfo(actor)->distance_to_leading_vehicle = distance;
}

void FastTrafficManagerLocal::SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
  GetActorInfo(actor)->percentage_ignore_walkers = perc;
}


void FastTrafficManagerLocal::SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
  GetActorInfo(actor)->percentage_ignore_vehicles = perc;
}


void FastTrafficManagerLocal::SetPercentageRunningLight(const ActorPtr &actor, const float perc) {
  GetActorInfo(actor)->percentage_running_light = perc;
}


void FastTrafficManagerLocal::SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
  GetActorInfo(actor)->percentage_running_sign = perc;
}


void FastTrafficManagerLocal::SetSynchronousMode(bool mode) {
  global_parameters.synchronous_mode = mode;
}


void FastTrafficManagerLocal::SetSynchronousModeTimeOutInMiliSecond(double time) {
  global_parameters.synchronous_timeout = std::chrono::duration<double, std::milli>(time);
}


void FastTrafficManagerLocal::SetGlobalDistanceToLeadingVehicle(const float dist) {
  global_parameters.distance_to_leading_vehicle = dist;
}


void FastTrafficManagerLocal::SetKeepRightPercentage(const ActorPtr &actor,const float percentage) {
  GetActorInfo(actor)->percentage_keep_right = percentage;

}


void FastTrafficManagerLocal::SetHybridPhysicsMode(const bool mode) {
  global_parameters.hybrid_physics_mode = mode;
}


void FastTrafficManagerLocal::SetHybridPhysicsRadius(const float radius) {
  global_parameters.hybrid_physics_radius = radius;
}


void FastTrafficManagerLocal::SetRandomDeviceSeed(const uint64_t seed) { /* TODO */ }


void FastTrafficManagerLocal::SetOSMMode(const bool mode_switch) {
  global_parameters.osm_mode = mode_switch;
}

std::shared_ptr<FastTrafficManagerLocal::ActorInfo> FastTrafficManagerLocal::GetActorInfo(const ActorPtr &actor) {
  std::lock_guard<std::mutex> lock(actor_info_mutex);
  auto i = actor_info.find(actor->GetId());
  if (i != actor_info.end())
    return i->second;
  return actor_info[actor->GetId()] = std::make_shared<ActorInfo>();
}


FastTrafficManagerLocal::ActorInfo FastTrafficManagerLocal::ActorInfo::update_and_reset(const GlobalParameters & global) {
  FastTrafficManagerLocal::ActorInfo r = *this;
  if (percentage_speed_difference < 0.f)
    r.percentage_speed_difference = global.percentage_speed_difference;
  if (distance_to_leading_vehicle < 0.f)
    r.distance_to_leading_vehicle = global.distance_to_leading_vehicle;
  // Reset the one-time flags
  percentage_keep_right = -1.f;
  force_lane_change  = {false, false};
}

void FastTrafficManagerLocal::Run() {
  auto world = cc::World(episode_proxy);
  auto local_map = std::make_shared<InMemoryMap>(world.GetMap());
  local_map->SetUp();


  TIMER("FastTrafficManagerLocal");
  while (run_traffic_manger.load()) {
    TIC;
    
    bool synchronous_mode = global_parameters.synchronous_mode;
    bool hybrid_physics_mode = global_parameters.hybrid_physics_mode;
    TOC("param");

    // Wait for external trigger to initiate cycle in synchronous mode.
    if (synchronous_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      step_begin_trigger.wait(lock, [this]() {return step_begin.load() || !run_traffic_manger.load();});
      step_begin.store(false);
    }
    TOC("sync");
    
    std::unordered_map<ActorId, ActorInfo> updated_actor_info;
    {
      // Fetch all actor info and apply the global parameters
      std::lock_guard<std::mutex> lock(actor_info_mutex);
      for(auto e: actor_info)
        updated_actor_info[e.first] = e.second->update_and_reset(global_parameters);
    }
    TOC("updated_actor_info");
    
    std::unique_lock<std::mutex> registration_lock(registration_mutex);
    TOC("registration_lock");
    
    // Updating simulation state, actor life cycle and performing necessary cleanup.
    alsm.Update(world);
    TOC("alsm.Update");

    // TODO: Main TM parts

    
    registration_lock.unlock();
    TOC("registration_lock.unlock");
    
    // Sending the current cycle's batch command to the simulator.
    if (synchronous_mode) {
      // If TCP and RPC preserve the call order then ApplyBatchSync is overkill here.
      // We do not need the response and only require the control_frame to be executed
      // before any other client commands
//       episode_proxy.Lock()->ApplyBatch(control_frame, false);
      step_end.store(true);
      step_end_trigger.notify_one();
    } else {
//       episode_proxy.Lock()->ApplyBatch(control_frame, false);
    }
    TOC("end");
    FINISH;
  }
}

} // namespace traffic_manager
} // namespace carla
