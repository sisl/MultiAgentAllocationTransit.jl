module MultiAgentAllocationTransit

import Base: isempty

# Base
using Random
using LinearAlgebra

# Stdlib
using DataStructures
using StaticArrays

# Registry repos
using Distributions
using Parameters
using JSON
using CSV
using DataFrames
using Distances
using NearestNeighbors
using IterTools
using GLPK
using JuMP
using LightGraphs

# Custom
using TOML
using Graphs
using MultiAgentPathFinding

# Types
export
    Location2D,
    LatLonCoords,
    convert_to_vector,
    vector_type,
    CityParams,
    parse_city_params,
    DroneParams,
    parse_drone_params,
    TransitGraph,
    OffTransitGraph,
    MAPFTransitEnv,
    MAPFTransitState,
    MAPFTransitVertexState,
    MAPFTransitAction,
    MAPFTransitConflict,
    MAPFTransitConstraints,
    AgentTask

# Utils
export
    distance_lat_lon_euclidean,
    is_in_bounds,
    hhmmss_to_seconds,
    generate_sites,
    load_depots,
    load_stop_to_location,
    load_transit_routes_single_time,
    EuclideanLatLong,
    stop_locations_nearest_neighbors,
    trip_meta_graph_fws_dists,
    get_stop_idx_to_trip_ids,
    get_non_dominated_trip_points

# GTFS Parser
export
    generate_stop_file,
    generate_trip_file


# Preprocessing
export
    trip_meta_graph_fws_dists,
    stop_locations_nearest_neighbors,
    get_stop_idx_to_trip_ids,
    generate_depot_to_sites_dists,
    true_stop_to_locations

# Task Allocation
export
    min_connecting_tour,
    get_connected_depots,
    add_merged_depot_edges!,
    get_multiedge_eulerian_tour

# Load Transit Env
export
    load_depot_site_locations_latlong,
    load_off_transit_graph_latlong,
    load_transit_graph_latlong,
    setup_state_graph


include("types.jl")
include("utils.jl")
include("gtfs_parser.jl")
include("preprocessing.jl")
include("task_allocation.jl")
include("mapf_transit.jl")
include("load_transit_env.jl")

end # module
