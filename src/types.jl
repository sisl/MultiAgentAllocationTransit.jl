const Location2D = SVector{2, Float64}
const LatLongCoords = NamedTuple{(:lat, :lon)}

@with_kw struct RouteWaypoint{TD <: Distribution}
    stop_id::Int64
    eta_dist::TD
end
const WaypointUniform = RouteWaypoint{Uniform{Float64}}

@with_kw struct OffTransitGraph{LOC}
    depots::Vector{LOC} = Vector{LOC}(undef, 0)
    sites::Vector{LOC}  = Vector{LOC}(undef, 0)
end

# The integer keys are consistent across routes and metadata
@with_kw struct TransitGraph{R <: RouteWaypoint, LOC}
    stop_to_location::Dict{Int64,LOC}        = Dict{Int64,LOC}()             # Maps Stop ID to location
    transit_routes::Vector{Vector{R}}            = Vector{Vector{R}}(undef, 0) # Always in chronological order
end

@with_kw struct ProblemGraph{OTG <: OffTransitGraph, TG <: TransitGraph}
    off_transit_graph::OTG
    transit_graph::TG
    vtx_id_graph::SimpleVListGraph{String}  # Vertex IDs are d-1 etc, s-1 etc, r-1-1 etc.
    depot_sites_to_vtx::Dict{String,Int64}  # Maps depot and site IDs to their vertex ID in graph
end

# Split by depot or site (Location) OR toute point (RouteWayPoint)
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
