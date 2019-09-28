"""
    augmented_trip_meta_graph_fws_dists(tg::TG, dist_fn::Function,
                                        n_depots::Int64, n_sites::Int64,
                                        depot_sites::Vector{LOC}, drone_params::DroneParams) where {LOC, TG <: TransitGraph}

Get distance matrix from Floyd-Warshall Shortest Paths alg on the augmented
trip metagraph. See Appendix II-B for details.
"""
function augmented_trip_meta_graph_fws_dists(tg::TG, dist_fn::Function,
                                             n_depots::Int64, n_sites::Int64,
                                             depot_sites::Vector{LOC}, drone_params::DroneParams) where {LOC, TG <: TransitGraph}

    n_trips = length(tg.transit_trips)
    n_verts = n_depots + n_sites + n_trips

    # Create the FWS dists matrix
    fws_dists = Inf*ones(n_verts, n_verts)

    # First do depots-sites
    for i = 1:n_depots + n_sites
        for j = 1:n_depots + n_sites

            if i != j
                dist = dist_fn(depot_sites[i], depot_sites[j])
                if dist < drone_params.max_distance
                    fws_dists[i, j] = dist
                    fws_dists[j, i] = dist
                end
            end
        end
    end

    # Now do depot-sites to trips
    for i = 1:n_depots + n_sites
        for j = n_depots+n_sites+1:n_verts
            trip = tg.transit_trips[j - n_depots - n_sites]
            mindist = Inf

            for rwp in trip

                dist = dist_fn(depot_sites[i], tg.stop_to_location[rwp.stop_id])
                mindist = (dist < mindist) ? dist : mindist

            end

            fws_dists[i, j] = mindist
            fws_dists[j, i] = mindist
        end
    end


    # Finally, do trip-trip
    for i = n_depots + n_sites + 1:n_verts
        trip1 = tg.transit_trips[i - n_depots - n_sites]
        for j = n_depots + n_sites + 1:n_verts
            trip2 = tg.transit_trips[j - n_depots - n_sites]
            mindist = Inf

            # Iterate over both trips and compute min distance
            for rwp1 in trip1
                for rwp2 in trip2

                    dist = dist_fn(tg.stop_to_location[rwp1.stop_id], tg.stop_to_location[rwp2.stop_id])

                    if drone_params.avg_speed * (rwp2.arrival_time - rwp1.arrival_time) > dist && dist < mindist
                        mindist = dist
                    end
                end
            end

            fws_dists[i, j] = mindist

        end
    end


    # Run FWS to get distances
    floyd_warshall!(fws_dists)

    return fws_dists
end


"""
    get_stop_idx_to_trip_ids(tg::TG) where {TG <: TransitGraph}

One-to-many mapping from stop IDs to trip IDs passing through them
"""
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


"""
    true_stop_to_locations(stop_to_location::Dict{Int64,LOC}, stop_idx_to_trips::Dict) where {LOC}

Filter out any stops that are not actually used by any trip.
"""
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


"""
    generate_city_halton_nn(city_params::CityParams; n_points::Int64 = 100,
                        bases::Vector{Int64} = [2, 3], discard::Int64 = 0)

Sample a halton sequence of points in a city and create a NNTree with them.
"""
function generate_city_halton_nn(city_params::CityParams; n_points::Int64 = 100,
                                 bases::Vector{Int64} = [2, 3], discard::Int64 = 0)

    lower = [city_params.lat_start, city_params.lon_start]
    upper = [city_params.lat_end, city_params.lon_end]

    # Generate the halton sequence
    city_halton_points = get_halton_sequence(n_points, bases, lower, upper, discard)

    # Generate the NN tree
    # nn_tree.data has the map from idx to point
    halton_nn_tree = BallTree(city_halton_points, EuclideanLatLong())

    return halton_nn_tree, city_halton_points
end

"""
    get_travel_time_estimate(halton_nn_tree::BallTree, loc1::LOC, loc2::LOC,
                             estimate_matrix::Matrix{Float64}) where LOC

Using the surrogate travel time estimate matrix, compute the
estimate between any two locations by looking up their corresponding nearest rep. location.
"""
function get_travel_time_estimate(halton_nn_tree::BallTree, loc1::LOC, loc2::LOC,
                                  estimate_matrix::Matrix{Float64}) where LOC

    # Get the nearest neighbor for each location
    # IMP - If they are the same, return 0.0
    loc1_vect = convert_to_vector(loc1)
    loc1_idxs, _ = knn(halton_nn_tree, loc1_vect, 1)
    loc1_idx = loc1_idxs[1]

    loc2_vect = convert_to_vector(loc2)
    loc2_idxs, _ = knn(halton_nn_tree, loc2_vect, 1)
    loc2_idx = loc2_idxs[1]

    # Returns 0.0 if same!
    return estimate_matrix[loc1_idx, loc2_idx]
end

"""
    compute_all_pairs_estimates(env::MAPFTransitEnv, n_halton_points::Int64, weight::Float64)

Preprocessing. Compute the pairwise travel times between pairs of Voronoi sites. See Appendix III
"""
function compute_all_pairs_estimates(env::MAPFTransitEnv, n_halton_points::Int64, weight::Float64)

    travel_time_estimates = zeros(n_halton_points, n_halton_points)

    for i = 1:n_halton_points
        println("From point $i:")
        for j = 1:n_halton_points

            if i != j
                orig_str = string("d-", i)
                goal_str = string("d-", j)

                travel_time_estimates[i, j] = get_depot_to_site_travel_time(env, weight, orig_str, goal_str)
            end
        end
    end

    return travel_time_estimates
end
