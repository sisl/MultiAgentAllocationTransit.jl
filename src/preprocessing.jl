## Takes in a transit graph
## Computes the minimum flight distance between each pair of trips
## Then runs FWS on the Time-Invariant Meta Route graph
function trip_meta_graph_fws_dists(tg::TG, dist_fn::Function) where {TG <: TransitGraph}

    n_trips = length(tg.transit_trips)

    # Create the Floyd-Warshall dists matrix
    dists = zeros(n_trips, n_trips)

    # Compute the min traversal distance between trips
    for i = 1:n_trips
        trip1 = tg.transit_trips[i]
        for j = 1:n_trips

            if i >= j
                dists[i, j] = dists[j, i]
                continue
            end

            trip2 = tg.transit_trips[j]

            mindist = Inf

            # Iterate over both trips and compute min distance
            for rwp1 in trip1
                for rwp2 in trip2

                    dist = dist_fn(tg.stop_to_location[rwp1.stop_id], tg.stop_to_location[rwp2.stop_id])

                    if dist < mindist
                        mindist = dist
                    end
                end
            end

            @assert mindist != Inf "Min dist for trips $i and $j is Inf!"
            dists[i, j] = mindist
        end
    end

    floyd_warshall!(dists)

    return dists

end


## Creates a nearest neighbor tree with stop locations
## Returns the index from nn row to stop ID
function stop_locations_nearest_neighbors(stop_to_location::Dict{Int64,LOC}, metric::M) where {LOC, M <: Metric}

    n_stops = length(collect(keys(stop_to_location)))
    nn_idx_to_stop = Vector{Int64}(undef, n_stops)

    VType = vector_type(LOC)

    data = Vector{VType}(undef, n_stops)

    # Assume stop_to_location keys are integers
    for (idx, stop_id) in enumerate(collect(keys(stop_to_location)))

        data[idx] = convert_to_vector(stop_to_location[stop_id])
        nn_idx_to_stop[idx] = stop_id

    end

    stops_nn_tree = BallTree(data, metric)

    return stops_nn_tree, nn_idx_to_stop

end # function


function get_stop_idx_to_trip_ids(tg::TG) where {TG <: TransitGraph}

    stop_idx_to_trips = Dict{Int64,Set{Int64}}()

    for (trip_id, trip) in enumerate(tg.transit_trips)

        for rp in trip
            @assert haskey(tg.stop_to_location, rp.stop_id)
            if ~(haskey(stop_idx_to_trips, rp.stop_id))
                stop_idx_to_trips[rp.stop_id] = Set{Int64}()
            end
            push!(stop_idx_to_trips[rp.stop_id], trip_id)
        end

    end

    return stop_idx_to_trips
end



function true_stop_to_locations(stop_to_location::Dict{Int64,LOC}, stop_idx_to_trips::Dict) where {LOC}

    keys_to_keep = Set{Int64}()

    true_stop_to_locs = Dict{Int64,LOC}()

    for stlkey in collect(keys(stop_to_location))

        if haskey(stop_idx_to_trips, stlkey)
            push!(keys_to_keep, stlkey)
        end
    end

    # Now copy over
    for kk in keys_to_keep
        true_stop_to_locs[kk] = stop_to_location[kk]
    end

    return true_stop_to_locs

end


# Compute the minimum pairwise distance between depots and sites either through direct flight
# OR by using the Time Invariant Route Graph
function generate_depot_to_sites_dists(otg::OffTransitGraph, tg::TransitGraph, stops_nn_tree::NNTree,
                                       nn_idx_to_stop::Vector{Int64}, stop_idx_to_trips::Dict{Int64,Set{Int64}},
                                       trips_fws_dists::Matrix{Float64}, dist_fn::Function)

    depot_to_sites_dists = zeros(length(otg.depots), length(otg.sites))

    for (d, depot_loc) in enumerate(otg.depots)
        for (s, site_loc) in enumerate(otg.sites)

            depot_vect = convert_to_vector(depot_loc)
            d_nn_idxs, d_nn_dists = knn(stops_nn_tree, depot_vect, 1)
            d_nn_idx = d_nn_idxs[1]
            d_nn_dist = d_nn_dists[1]
            d_nn_stop = nn_idx_to_stop[d_nn_idx]

            site_vect = convert_to_vector(site_loc)
            s_nn_idxs, s_nn_dists = knn(stops_nn_tree, site_vect, 1)
            s_nn_idx = s_nn_idxs[1]
            s_nn_dist = s_nn_dists[1]
            s_nn_stop = nn_idx_to_stop[s_nn_idx]

            min_trip_to_trip = Inf
            for d_nn_trip_id in stop_idx_to_trips[d_nn_stop]
                for s_nn_trip_id in stop_idx_to_trips[s_nn_stop]
                    min_trip_to_trip = (trips_fws_dists[d_nn_trip_id, s_nn_trip_id] < min_trip_to_trip) ? trips_fws_dists[d_nn_trip_id, s_nn_trip_id] : min_trip_to_trip
                end
            end

            # Either direct flight OR through trips
            mindist = min(dist_fn(depot_loc, site_loc), d_nn_dist + s_nn_dist + min_trip_to_trip)
            depot_to_sites_dists[d, s] = mindist
        end
    end

    return depot_to_sites_dists

end


# TODO : Snapshot preprocessing
# For each depot/site-trip pair, find the set of non-dominated (by time/weight) reachable transit points
# IMP - this won't work when leaving a site since that time is unknown!!
