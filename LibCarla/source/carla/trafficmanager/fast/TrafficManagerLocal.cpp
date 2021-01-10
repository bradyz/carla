//
// Created by Philipp Krähenbühl on 1/9/21.
//

#include "carla/Sockets.h"
#include "carla/client/detail/Simulator.h"

#include "carla/trafficmanager/SnippetProfiler.h"
#include "carla/trafficmanager/fast/TrafficManagerLocal.h"
#include "carla/trafficmanager/fast/Collision.h"
#include "carla/trafficmanager/fast/Localization.h"
#include "carla/trafficmanager/fast/TrafficLight.h"

namespace carla {
namespace traffic_manager {

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

        // autopilot components
//
//          collision_stage(CollisionStage(alsm,
//                                         buffer_map,
//                                         track_traffic,
//                                         parameters,
//                                         collision_frame,
//                                         debug_helper,
//                                         random_devices)),
//          traffic_light_stage(TrafficLightStage(alsm,
//                                                buffer_map,
//                                                parameters,
//                                                world,
//                                                tl_frame,
//                                                random_devices)),
//          motion_plan_stage(MotionPlanStage(alsm,
//                                            parameters,
//                                            buffer_map,
//                                            track_traffic,
//                                            longitudinal_PID_parameters,
//                                            longitudinal_highway_PID_parameters,
//                                            lateral_PID_parameters,
//                                            lateral_highway_PID_parameters,
//                                            localization_frame,
//                                            collision_frame,
//                                            tl_frame,
//                                            world,
//                                            control_frame)),
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
  GetActorParameters(actor)->percentage_speed_difference = percentage;
}

void FastTrafficManagerLocal::SetGlobalPercentageSpeedDifference(float const percentage) {
  global_parameters.percentage_speed_difference = percentage;
}

/// Method to set collision detection rules between vehicles.
void FastTrafficManagerLocal::SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor,
                                                    const bool detect_collision) {
  auto p1 = GetActorParameters(reference_actor);
  auto p2 = GetActorParameters(other_actor);
  if (detect_collision) {
    p1->cannot_collide.erase(other_actor->GetId());
    p2->cannot_collide.erase(reference_actor->GetId());
  } else {
    p1->cannot_collide.insert(other_actor->GetId());
    p2->cannot_collide.insert(reference_actor->GetId());
  }
}

void FastTrafficManagerLocal::SetForceLaneChange(const ActorPtr &actor, const bool direction) {
  GetActorParameters(actor)->force_lane_change = {true, direction};
}

void FastTrafficManagerLocal::SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
  GetActorParameters(actor)->auto_lane_change = enable;
}

void FastTrafficManagerLocal::SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
  GetActorParameters(actor)->distance_to_leading_vehicle = distance;
}

void FastTrafficManagerLocal::SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
  GetActorParameters(actor)->percentage_ignore_walkers = perc;
}


void FastTrafficManagerLocal::SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
  GetActorParameters(actor)->percentage_ignore_vehicles = perc;
}


void FastTrafficManagerLocal::SetPercentageRunningLight(const ActorPtr &actor, const float perc) {
  GetActorParameters(actor)->percentage_running_light = perc;
}


void FastTrafficManagerLocal::SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
  GetActorParameters(actor)->percentage_running_sign = perc;
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
  GetActorParameters(actor)->percentage_keep_right = percentage;
}


void FastTrafficManagerLocal::SetHybridPhysicsMode(const bool mode) {
  global_parameters.hybrid_physics_mode = mode;
}


void FastTrafficManagerLocal::SetHybridPhysicsRadius(const float radius) {
  global_parameters.hybrid_physics_radius = radius;
}


void FastTrafficManagerLocal::SetRandomDeviceSeed(const uint64_t seed) {
  global_parameters.seed = seed;
}


void FastTrafficManagerLocal::SetOSMMode(const bool mode_switch) {
  global_parameters.osm_mode = mode_switch;
}

std::shared_ptr<ActorParameters> FastTrafficManagerLocal::GetActorParameters(const ActorPtr &actor) {
  std::lock_guard<std::mutex> lock(actor_parameters_mutex);
  auto i = actor_parameters.find(actor->GetId());
  if (i != actor_parameters.end())
    return i->second;
  return actor_parameters[actor->GetId()] = std::make_shared<ActorParameters>();
}


ActorParameters ActorParameters::update_and_reset(const GlobalParameters & global) {
  ActorParameters r = *this;
  if (percentage_speed_difference < 0.f)
    r.percentage_speed_difference = global.percentage_speed_difference;
  if (distance_to_leading_vehicle < 0.f)
    r.distance_to_leading_vehicle = global.distance_to_leading_vehicle;
  // Reset the one-time flags
  percentage_keep_right = -1.f;
  force_lane_change  = {false, false};
  return r;
}

void FastTrafficManagerLocal::Run() {
  auto world = cc::World(episode_proxy);
  auto local_map = std::make_shared<InMemoryMap>(world.GetMap());
  local_map->SetUp();

  FastLocalizationStage localization_stage = FastLocalizationStage(alsm, local_map);
  FastTrafficLightStage traffic_light_stage = FastTrafficLightStage(alsm, world);

  TIMER("FastTrafficManagerLocal");
  while (run_traffic_manger.load()) {
    TIC;

    bool synchronous_mode = global_parameters.synchronous_mode;
//    bool hybrid_physics_mode = global_parameters.hybrid_physics_mode;
    TOC("param");

    // Wait for external trigger to initiate cycle in synchronous mode.
    if (synchronous_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      step_begin_trigger.wait(lock, [this]() {return step_begin.load() || !run_traffic_manger.load();});
      step_begin.store(false);
    }
    TOC("sync");

    std::unordered_map<ActorId, ActorParameters> updated_actor_parameters;
    {
      // Fetch all actor info and apply the global parameters
      std::lock_guard<std::mutex> lock(actor_parameters_mutex);
      for(auto e: actor_parameters)
        updated_actor_parameters[e.first] = e.second->update_and_reset(global_parameters);
    }
    TOC("updated_actor_parameters");

    {
      std::lock_guard<std::mutex> registration_lock(registration_mutex);

      // Updating simulation state, actor life cycle and performing necessary cleanup.
      alsm.Update(world);
      TOC("alsm.Update");
    }

    // Run core operation stages.
    for (const  auto & p: updated_actor_parameters)
      localization_stage.Update(p.first, p.second, global_parameters);
    TOC("localization_stage");

    FastCollisionStage collision_stage(alsm, localization_stage.GetTrackTraffic());
    for (const  auto & p: updated_actor_parameters)
      collision_stage.Update(p.first, p.second, global_parameters);
    TOC("collision_stage");

    for (const  auto & p: updated_actor_parameters) {
      traffic_light_stage.Update(p.first, p.second, global_parameters);
    }

//
//    for (unsigned long index = 0u; index < n_vehicles; ++index) {
//      traffic_light_stage.Update(index);
//      motion_plan_stage.Update(index);
//    }
    TOC("traffic_light_stage");

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
