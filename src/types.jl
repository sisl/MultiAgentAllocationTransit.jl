const Location2D = SVector{2, Float64}
const LatLonCoords = NamedTuple{(:lat, :lon), Tuple{Float64,Float64}}
LatLonCoords() = (lat = 0.0, lon = 0.0)

function convert_to_vector(c::LatLonCoords)
    return Location2D(c.lat, c.lon)
end

vector_type(::Type{LatLonCoords}) = Location2D

@with_kw struct RouteWaypoint
    stop_id::Int64
    arrival_time::Float64
end


@with_kw struct OffTransitGraph{LOC}
    depots::Vector{LOC}
    sites::Vector{LOC}
end

# The integer keys are consistent across routes and metadata
@with_kw struct TransitGraph{LOC}
    stop_to_location::Dict{Int64,LOC}               # Maps Stop ID to location
    transit_trips::Vector{Vector{RouteWaypoint}}    # Always in chronological order
    transit_capacity::Vector{Int64}                 # Capacity for each trip - matches transit_trips
end

@with_kw struct CityParams
    lat_start::Float64
    lat_end::Float64
    lon_start::Float64
    lon_end::Float64
end

function parse_city_params(param_file::String)

    params_dict = TOML.parsefile(param_file)

    return CityParams(lat_start = params_dict["LATSTART"],
                      lat_end = params_dict["LATEND"],
                      lon_start = params_dict["LONSTART"],
                      lon_end = params_dict["LONEND"])
end


@with_kw struct DroneParams
    max_distance::Float64
    max_air_time::Float64
    avg_speed::Float64
    stop_buffer_time::Float64
end

function parse_drone_params(param_file::String)

    params_dict = TOML.parsefile(param_file)

    return DroneParams(max_distance = params_dict["MAX_DISTANCE"],
                       max_air_time = params_dict["MAX_AIR_TIME"],
                       avg_speed = params_dict["AVG_SPEED"],
                       stop_buffer_time = params_dict["STOP_BUFFER_TIME"])
end



# Strings are "d-1", "d-2" etc., "s-1", "s-2" etc and "r-1-1", "r-1-2", ... "r-2-1" etc.
@with_kw struct MAPFTransitState{LOC}
    time::Float64       # Planned time to be at the vertex. For routewaypoint, should be before ETA
    location::LOC              # The actual vertex string ID (TG or OTG); used with get_location_or_routept
end

Base.isequal(s1::MAPFTransitState, s2::MAPFTransitState) = (s1.time, s1.location) == (s2.time, s2.location)

@enum ActionType Stay=1 Fly=2 Board=3

# Empty struct - action implicit
struct MAPFTransitAction <: MAPFAction
    action::ActionType
end


# Board Conflict - Two drones go to the same ROUTE waypoint
# Capacity Conflict - A transit route has more drones than it can accommodate
@enum ConflictType Transfer=1 Capacity=2

@with_kw struct MAPFTransitConflict <: MAPFConflict
    type::ConflictType
    overlap_vertices::Set{Int64}
    agent_to_state_idx::Dict{Int64,Int64} # Maps overlapping agent ID to index along state vect
    cap::Int64                      = 1
end

@enum ConstraintSubPath towards=1 from=2

# Vertex Constraints are basically MAPFTransitState instances for depot/sites
struct MAPFTransitConstraints <: MAPFConstraints
    avoid_vertex_map::Dict{ConstraintSubPath,Set{Int64}}
end

MAPFTransitConstraints() = MAPFTransitConstraints(Dict{ConstraintSubPath,Set{Int64}}(towards => Set{Int64}(),
                                                                                     from => Set{Int64}()))

Base.isempty(mtc::MAPFTransitConstraints) = isempty(mtc.avoid_vertex_map[towards]) && isempty(mtc.avoid_vertex_map[from])

@with_kw mutable struct MAPFTransitVertexState{MTS <: MAPFTransitState} <: MAPFState
    idx::Int64
    state::MTS
    vertex_str::String
end

function reset_time(vs::MAPFTransitVertexState, time::Float64=0.0)
    vs.state = MAPFTransitState(time, vs.state.location)
end

function Graphs.vertex_index(g::SimpleVListGraph{MAPFTransitVertexState}, v::MAPFTransitVertexState)
    return v.idx
end

const AgentTask = NamedTuple{(:origin, :site, :dest)}


# Tracks the current state of the agent
@with_kw mutable struct AgentState
    task::AgentTask
    site_crossed::Bool              = false
    next_finish_time::Float64       = 0.0
    dist_flown::Float64             = 0.0
end


@with_kw mutable struct MAPFTransitEnv{OTG <: OffTransitGraph,
                                       TG <: TransitGraph, NN <: NNTree, MVTS <: MAPFTransitVertexState} <: MAPFEnvironment
    off_transit_graph::OTG
    transit_graph::TG
    state_graph::SimpleVListGraph{MVTS}          # Vertex IDs are d-1 etc, s-1 etc, r-1-1 etc.
    agent_states::Vector{AgentState}
    depot_sites_to_vtx::Dict{String,Int64}                          # Maps depot and site IDs to their vertex ID in graph - needed for start-goal IDXs
    trip_to_vtx_range::Vector{Tuple{Int64,Int64}}
    stops_nn_tree::NN                                               # Nearest neighbor tree for stop locations
    nn_idx_to_stop::Vector{Int64}                                   # Maps the ID from NNTree to stop id
    stop_idx_to_trips::Dict{Int64,Set{Int64}}                       # Maps the stop ID to trips passing through them
    trips_fws_dists::Matrix{Float64}
    depot_to_sites_dists::Matrix{Float64}
    drone_params::DroneParams
    dist_fn::Function
    curr_site_points::Vector{Int64}
    curr_goal_idx::Int64                                    = 0
    plan_ref_times::Vector{Float64}                         = Float64[]
    # Diagnostics
    valid_transit_options::Vector{Int64}                    = Int64[]
    any_invalid_path::Bool                                  = false
    valid_path_dists::Vector{Float64}                       = Float64[]
end


function get_vertex_location(env::MAPFTransitEnv, vtx_str::String)

    splitstr = split(vtx_str, "-")

    if splitstr[1] == "d"
        return env.off_transit_graph.depots[parse(Int64, splitstr[2])]
    elseif splitstr[1] == "s"
        return env.off_transit_graph.sites[parse(Int64, splitstr[2])]
    else
        rwp = env.transit_graph.transit_trips[parse(Int64, splitstr[2])][parse(Int64, splitstr[3])]
        return env.stop_to_location[rwp.stop_id]
    end

end
