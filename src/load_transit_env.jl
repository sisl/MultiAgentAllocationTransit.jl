## Define ALL the functions needed to load an environment
function load_depot_site_locations_latlong(filename::String)

    locations = Vector{LatLonCoords}(undef, 0)

    open(filename, "r") do f
        loc_dict = JSON.parse(f)

        for loc_id in sort(collect(keys(loc_dict)))
            loc = loc_dict[loc_id]
            push!(locations, LatLonCoords((lat = loc["lat"], lon = loc["lon"])))
        end
    end

    return locations
end


function load_off_transit_graph_latlong(depot_file::String, site_file::String)

    depots = load_depot_site_locations_latlong(depot_file)
    sites = load_depot_site_locations_latlong(site_file)

    return TransitGraph(depots, sites)
end


function load_transit_graph_latlong(stop_coords_file::String, trips_file::String,
                                    max_transit_capacity::Int64, rng::RNG) where {RNG <: AbstractRNG}

    # Load stop to locations
    stop_to_location = Dict{Int64,LatLonCoords}()

    open(stop_coords_file, "r") do f

        stop_coord_dict = JSON.parse(f)

        for (idx_str, latlon) in stop_coord_dict
            idx = parse(Int64, idx_str)
            coords = LatLonCoords((lat = latlon["lat"], lon = latlon["lon"]))
            stop_to_location[idx] = coords
        end
    end

    # Load trips
    transit_trips = Vector{Vector{RouteWaypoint}}(undef, 0)
    trips_dict = Dict()

    open(trips_file, "r") do f

        trips_dict = JSON.parse(f)

        for trip_id in sort(collect(keys(trips_dict)))

            trip = trips_dict[string(trip_id)]
            this_trip = Vector{RouteWaypoint}(undef, length(collect(keys(trip))))

            for seq = 1:length(collect(keys(trip)))
                this_trip[seq] = RouteWaypoint(stop_id = trip[string(seq)]["stop_id"],
                                               arrival_time = trip[string(seq)]["arrival_time"])
            end

            push!(transit_trips, this_trip)
        end
    end


    transit_capacity = [rand(rng, 1:max_transit_capacity) for _ =  1:length(transit_trips)]

    return TransitGraph(stop_to_location, transit_trips, transit_capacity)

end

# return the state_graph and depot_sites_to_vtx
function setup_state_graph(transit_graph::TG, off_transit_graph::OTG) where {TG <: TransitGraph, OTG <: OffTransitGraph}

    state_graph = SimpleVListGraph{MAPFTransitVertexState}()
    depot_sites_to_vtx = Dict{String,Int64}()

    # First add depots and sites
    for (d, depot_loc) in enumerate(off_transit_graph.depots)
        idx = num_vertices(state_graph) + 1
        vertex_str = string("d-", d)
        new_vtx = MAPFTransitVertexState(idx = idx, state = MAPFTransitState(time = 0.0, location = depot_loc),
                                                    vertex_str = vertex_str)
        add_vertex!(state_graph, new_vtx)
        depot_sites_to_vtx[vertex_str] = idx
    end

    for (s, site_loc) in enumerate(off_transit_graph.sites)
        idx = num_vertices(state_graph) + 1
        vertex_str = string("s-", s)
        new_vtx = MAPFTransitVertexState(idx = idx, state = MAPFTransitState(time = 0.0, location = site_loc),
                                         vertex_str = vertex_str)
        add_vertex!(state_graph, new_vtx)
        depot_sites_to_vtx[vertex_str] = idx
    end

    # Now iterate over trips
    trip_to_vtx_range = Vector{Tuple{Int64,Int64}}(undef, length(transit_graph.transit_trips))

    for (trip_id, trip) in enumerate(transit_graph.transit_trips)

        range_st = num_vertices(state_graph) + 1
        range_end = range_st

        # Add a vertex for each route waypoint
        for (seq, rwp) in enumerate(trip)

            idx = range_end
            state = MAPFTransitState(time = rwp.arrival_time, location = transit_graph.stop_to_location[rwp.stop_id])
            vertex_str = string("r-", trip_id, "-", seq)

            new_vtx = MAPFTransitVertexState(idx = idx, state = state, vertex_str = vertex_str)
            add_vertex!(state_graph, new_vtx)
            range_end += 1

        end

        trip_to_vtx_range[trip_id] = (range_st, range_end-1)
    end

    return state_graph, depot_sites_to_vtx, trip_to_vtx_range
end
