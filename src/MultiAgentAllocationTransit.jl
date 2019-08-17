module MultiAgentAllocationTransit

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

# Custom
using TOML
using Graphs
using MultiAgentPathFinding

# Types
export
    Location2D,
    LatLongCoords,
    convert_to_vector,
    vector_type,
    CityParams,
    parse_city_params,
    TransitGraph,
    OffTransitGraph,
    MAPFTransitEnv

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
    get_stop_idx_to_trip_ids

# GTFS Parser
export
    generate_stop_file,
    generate_trip_file

include("types.jl")
include("utils.jl")
include("gtfs_parser.jl")
include("preprocessing.jl")
include("mapf_transit.jl")

end # module
