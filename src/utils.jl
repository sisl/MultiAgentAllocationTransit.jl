## General utilities used across the board

"""
Compute the Euclidean distance between two lat-long coordinates close to each other.
Distance in km
"""
function distance_lat_lon_euclidean(coords1::LatLonCoords, coords2::LatLonCoords)
    deglen = 110.25
    x = coords1.lat - coords2.lat
    y = (coords1.lon - coords2.lon)*cos(coords2.lat)
    return deglen*sqrt(x^2 + y^2)
end


function is_in_bounds(params::CityParams, coords::LatLonCoords)

    return (coords.lat >= params.lat_start && coords.lat <= params.lat_end) &&
            (coords.lon >= params.lon_start && coords.lon <= params.lon_end)
end

function hhmmss_to_seconds(time_str::String)

    str_arr = split(time_str, ":")
    int_arr = [parse(Int64, s) for s in str_arr]
    time_arr = [3600., 60., 1.]

    return dot(int_arr, time_arr)
end

## coords_bb - low and hi are the lower and upper corners respectively
function generate_sites(coords_bb_lo::LatLonCoords, coords_bb_hi::LatLonCoords,
                        n_sites::Int64, rng::RNG) where {RNG <: AbstractRNG}

    sites = Vector{LatLonCoords}(undef, n_sites)

    for i = 1:n_sites

        lat_dist = Uniform(coords_bb_lo.lat, coords_bb_hi.lat)
        lon_dist = Uniform(coords_bb_lo.lon, coords_bb_hi.lon)

        sites[i] = LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist)))

    end

    return sites

end # function

function get_non_dominated_trip_points(env::MAPFTransitEnv, avoid_vertex_idxs::Set{Int64},
                                       vtx::MAPFTransitVertexState, tid::Int64)

    trip_vtx_range = env.trip_to_vtx_range[tid]

    time_dist_set = Vector{Tuple{Int64,Float64,Float64}}(undef, 0)

    # First run through and put in list if reachable by agent
    for seq = trip_vtx_range[1]:trip_vtx_range[2]
        dist = env.dist_fn(vtx.state.location, env.state_graph.vertices[seq].state.location)
        tdiff = env.state_graph.vertices[seq].state.time - vtx.state.time

        if dist < env.drone_params.max_distance && env.drone_params.avg_speed * tdiff > dist &&
            ~(seq in avoid_vertex_idxs)
            push!(time_dist_set, (seq, tdiff, dist))
        end
    end

    if isempty(time_dist_set)
        return []
    end
    # YOU KNOW THIS IS ALREADY SORTED IN INCREASING ETA AND REACHABLE
    non_dom_idxs = Set{Int64}(1)

    # Start from second element
    for (i, elem) in enumerate(time_dist_set[2:end])
        dom = false
        for ndi in non_dom_idxs
            # If dist is worse, then dominated
            if elem[3] >= time_dist_set[ndi][3]
                dom = true
                break
            end
        end

        if dom == false
            push!(non_dom_idxs, i)
        end
    end

    # Now return all seq points from ndi
    seqs = [time_dist_set[ndi][1] for ndi in non_dom_idxs]


end # function

# Reads in a depot file and returns a vector of depot locations
function load_depots(depot_file::String)

    open(depot_file, "r") do df
        depot_dict = JSON.parse(df)
    end

    n_depots = length(collect(keys(depot_dict)))

    depots = Vector{LatLonCoords}(undef, n_depots)

    # Enforcing keys to be integers
    for (idx_str, latlon) in depot_dict
        idx = parse(Int64, id_str)
        depots[idx] = LatLonCoords((lat = latlon["lat"], lon = latlon["lon"]))
    end

    return depots
end


function load_stop_to_location(::Type{LatLonCoords}, stop_coords_file::String)

    stop_coord_dict = Dict()
    open(stop_coords_file, "r") do f
        stop_coord_dict = JSON.parse(f)
    end

    stop_to_location = Dict{Int64,LatLonCoords}()

    for (id_str, latlon) in stop_coord_dict
        stop_to_location[parse(Int64, id_str)] = LatLonCoords((lat = latlon["lat"], lon = latlon["lon"]))
    end

    return stop_to_location
end


## Loads the transit routes from the trips file
## get_eta_dist creates a Distribution from the individual ETA in the file
function load_transit_routes_single_time(trip_file::String)

    trips_dict = Dict()
    open(trip_file, "r") do f
        trips_dict = JSON.parse(f)
    end

    n_trips = length(collect(keys(trips_dict)))
    transit_trips = Vector{Vector{RouteWaypoint}}(undef, n_trips)

    for trip_num = 1:n_trips

        trip = trips_dict[string(trip_num)]
        n_stops = length(collect(keys(trip)))

        this_trip = Vector{RouteWaypoint}(undef, n_stops)

        for stop_seq = 1:n_stops

            # Cross-reference stop sequence from trip dictionary
            id_time = trip[string(stop_seq)]

            # Get info for route waypoint
            waypt = RouteWaypoint(stop_id = id_time["stop_id"],
                                  arrival_time = id_time["arrival_time"])

            this_trip[stop_seq] = waypt
        end

        transit_trips[trip_num] = this_trip
    end

    return transit_trips
end


## For nearestneighbor metric
struct EuclideanLatLong <: Metric
end

function Distances.evaluate(::EuclideanLatLong,
                            x::Union{SVector{2,Float64}, MVector{2,Float64}},
                            y::Union{SVector{2,Float64}, MVector{2,Float64}})
    coords1 = (lat=x[1], lon=x[2])
    coords2 = (lat=y[1], lon=y[2])
    return distance_lat_lon_euclidean(coords1, coords2)
end
