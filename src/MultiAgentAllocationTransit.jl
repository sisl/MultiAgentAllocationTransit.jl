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
GLPK.jl_set_preemptive_check(false)
using JuMP
using LightGraphs

# Custom
using TOML
using Graphs
using MultiAgentPathFinding

# for plotting
using Plots

# Types
export
    Location2D,
    LatLonCoords,
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
    AgentTask,
    AgentState

# Utils
export
    distance_lat_lon_euclidean,
    generate_sites,
    load_depots,
    load_stop_to_location,
    load_transit_routes_single_time,
    EuclideanLatLong,
    get_non_dominated_trip_points,
    get_halton_sequence,
    plot_depots_sites!,
    render_drones

# GTFS Parser
export
    generate_stop_file,
    generate_trip_file


# Preprocessing
export
    get_stop_idx_to_trip_ids,
    true_stop_to_locations,
    transit_graph_preprocessing,
    generate_city_halton_nn,
    get_travel_time_estimate,
    compute_all_pairs_estimates,
    augmented_trip_meta_graph_fws_dists

# Task Allocation
export
    task_allocation,
    get_agent_task_set,
    get_next_agent_task

# Load Transit Env
export
    load_depot_site_locations_latlong,
    load_off_transit_graph_latlong,
    load_transit_graph_latlong,
    setup_state_graph


# MAPF Transit
export
    allocation_cost_fn_wrapper,
    allocation_cost_wrapper_estimate,
    update_agent_states!,
    get_first_finish,
    get_replan_constraints,
    replan_individual!,
    replan_collective!,
    set_solution_diagnostics!,
    get_depot_to_site_travel_time


include("types.jl")
include("utils.jl")
include("gtfs_parser.jl")
include("preprocessing.jl")
include("task_allocation.jl")
include("mapf_transit.jl")
include("load_transit_env.jl")

end # module
