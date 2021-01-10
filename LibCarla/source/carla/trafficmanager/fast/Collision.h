
#pragma once

#include <memory>

#include "boost/geometry.hpp"
#include "boost/geometry/geometries/geometries.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"

#include "carla/client/DebugHelper.h"

#include "carla/trafficmanager/DataStructures.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/fast/ALSM.h"

namespace carla {
namespace traffic_manager {

struct GeometryComparison {
  float reference_vehicle_to_other_geodesic;
  float other_vehicle_to_reference_geodesic;
  float inter_geodesic_distance;
  float inter_bbox_distance;
};

namespace cc = carla::client;
namespace bg = boost::geometry;

using LocationVector = std::vector<cg::Location>;
using GeodesicBoundaryMap = std::unordered_map<ActorId, LocationVector>;
using GeometryComparisonMap = std::unordered_map<uint64_t, GeometryComparison>;
using Polygon = bg::model::polygon<bg::model::d2::point_xy<double>>;

struct GlobalParameters;
struct ActorParameters;

class FastCollisionStage {
  FastALSM & alsm;
  const TrackTraffic & track_traffic;
  // Structures to cache geodesic boundaries of vehicle and
  // comparision between vehicle boundaries
  // to avoid repeated computation within a cycle.
  GeometryComparisonMap geometry_cache;
  GeodesicBoundaryMap geodesic_boundary_map;

  // Method to determine if a vehicle is on a collision path to another.
  std::pair<bool, float> NegotiateCollision(ActorState & reference,
                                            const ActorState & other,
                                            const uint64_t reference_junction_look_ahead_index);

  // Method to calculate bounding box extention length ahead of the vehicle.
  float GetBoundingBoxExtention(const ActorState & actor);

  // Method to calculate polygon points around the vehicle's bounding box.
  LocationVector GetBoundary(const ActorState & actor);

  // Method to construct polygon points around the path boundary of the vehicle.
  LocationVector GetGeodesicBoundary(const ActorState & actor);

  Polygon GetPolygon(const LocationVector &boundary);

  // Method to compare path boundaries, bounding boxes of vehicles
  // and cache the results for reuse in current update cycle.
  GeometryComparison GetGeometryBetweenActors(const ActorState & reference,
                                              const ActorState & other);

public:
  FastCollisionStage(FastALSM & alsm, const TrackTraffic & track_traffic);

  void Update(ActorId actor_id,
              const ActorParameters &parameters,
              const GlobalParameters &global_parameters);
};

} // namespace traffic_manager
} // namespace carla
