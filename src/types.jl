const Location2D = SVector{2, Float64}
const LatLongCoords = NamedTuple{(:lat, :lon)}

function convert_to_vector(c::LatLongCoords)
    return Location2D(c.lat, c.lon)
end

vector_type(::Type{LatLongCoords}) = Location2D

@with_kw struct RouteWaypoint
    stop_id::Int64
    arrival_time::Float64
end


@with_kw struct OffTransitGraph{LOC}
    depots::Vector{LOC} = Vector{LOC}(undef, 0)
    sites::Vector{LOC}  = Vector{LOC}(undef, 0)
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




@with_kw struct MAPFTransitState <: MAPFState
    time::Float64       # Planned time to be at the vertex. For routewaypoint, should be before ETA
    vertex_str::String  # The actual vertex string ID (TG or OTG); used with get_location_or_routept
end

# Empty struct - action implicit
struct MAPFTransitAction <: MAPFAction
end


# Board Conflict - Two drones go to the same ROUTE waypoint
# Capacity Conflict - A transit route has more drones than it can accommodate
@enum ConflictType Board=1 Capacity=2

@with_kw struct MAPFTransitConflict <: MAPFConflict
    time::Float64
    type::ConflictType
    overlaps::Dict{Int64,Tuple{String,String}}  # Maps agent ID to overlapping subroutes
end

# Vertex Constraints are basically MAPFTransitState instances for depot/sites
struct MAPFTransitConstraints
    avoid_vertex_strs::Set{String}
end

@with_kw struct MAPFTransitVertexState
    idx::Int64
    state::MAPFTransitState
end

function Graphs.vertex_index(g::SimpleVListGraph{MAPFTransitVertexState}, v::MAPFTransitVertexState)
    return v.idx
end

@with_kw mutable struct MAPFTransitEnv{OTG <: OffTransitGraph, TG <: TransitGraph, NN <: NNTree}
    off_transit_graph::OTG
    transit_graph::TG
    state_graph::SimpleVListGraph{MAPFTransitVertexState}          # Vertex IDs are d-1 etc, s-1 etc, r-1-1 etc.
    depot_sites_to_vtx::Dict{String,Int64}                          # Maps depot and site IDs to their vertex ID in graph - needed for start-goal IDXs
    stops_nn_tree::NN                                               # Nearest neighbor tree for stop locations
    nn_idx_to_stop::Vector{Int64}                                   # Maps the ID from NNTree to stop id
    stop_idx_to_trips::Dict{Int64,Set{Int64}}                       # Maps the stop ID to trips passing through them
end



# Split by depot or site (Location) OR route point (RouteWayPoint)
function get_location_or_routept(pg::ProblemGraph, vtx_id::String)

    splitstr = split(vtx_id, "-")
    if splitstr[1] == "d"
        return pg.off_transit_graph.depots[parse(Int64, splitstr[2])]
    elseif splitstr[1] == "s"
        return pg.off_transit_graph.sites[parse(Int64, splitstr[2])]
    else
        return pg.transit_graph.transit_routes[parse(Int64, splitstr[2])][parse(Int64, splitstr[3])]
    end
end
