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
