function generate_stop_file(stop_txt::String, params::CityParams, out_file::String)

    stop_file = CSV.File(stop_txt)

    n_stops = length(stop_file.stop_id)
    stops_dict = Dict{Int64,LatLonCoords}()

    for (i, id_str) in enumerate(stop_file.stop_id)
        id = 0
        if ismissing(id_str)
            @info "Stop missing, continuing!"
        else
            if typeof(id_str) == String
                try
                    id = parse(Int64, id_str)
                catch ArgumentError
                    @info "Found non-integer stop id $(id_str); ignoring!"
                    continue
                end
            else
                id = id_str
            end

            @assert haskey(stops_dict, id) == false
            coords = LatLonCoords((lat = stop_file.stop_lat[i], lon = stop_file.stop_lon[i]))

            if is_in_bounds(params, coords)
                stops_dict[id] = coords
            end # if
        end
    end

    open(out_file, "w") do stop_out_file
        JSON.print(stop_out_file, stops_dict, 2)
    end

    return stops_dict

end

## Loop through route IDs
## Choose any one trip randomly with that route ID - note TRIP ID
## Go to stop_times, look up stop sequence and STOP IDs for the trip, note TIMES
## Ignore trips with any stops outside bounding box
## Save trips with zero-relative time in another file
function generate_trip_file(route_txt::String, trip_txt::String,
                            stop_time_txt::String, stop_coords::Dict{Int64,LatLonCoords},
                            out_file::String, rng::RNG) where {RNG <: AbstractRNG}

    route_df = CSV.read(route_txt)
    trip_df = CSV.read(trip_txt)
    stop_time_df = CSV.read(stop_time_txt)

    all_trips_dict = Dict()
    out_trip_idx = 1
    trip_points = 0

    max_rel_time = 0.0

    for rid in route_df[:, :route_id]

        @info rid
        # Filter trips for route ID
        # Choose a trip ID at random
        trips_for_route = trip_df[trip_df.route_id .== rid, :trip_id]
        tid = trips_for_route[rand(rng, 1:end)]

        # Now look at the stop_times for that trip ID
        trip_dict = Dict{Int64,Dict}()

        # Get the stop sequence for that trip
        trip_stop_seq = stop_time_df[stop_time_df.trip_id .== tid, :]

        # Don't assume they are necessarily vertical
        n_stops = DataFrames.nrow(trip_stop_seq)
        is_valid_trip = true

        # Get base time
        base_time_str = trip_stop_seq[trip_stop_seq.stop_sequence .== 1, :arrival_time]
        base_time = hhmmss_to_seconds(base_time_str[1])

        for stop_seq = 1:n_stops

            # Get row for that stop sequence
            stop_row = trip_stop_seq[trip_stop_seq.stop_sequence .== stop_seq, :]

            stop_id = 0
            if typeof(stop_row.stop_id[1]) == String
                try
                    stop_id = parse(Int64, stop_row.stop_id[1])
                catch ArgumentError
                    @info "Found non-integer stop id $(stop_row.stop_id[1]), ignoring!"
                    is_valid_trip = false
                    break
                end
            else
                stop_id = stop_row.stop_id[1]
            end

            if ~(haskey(stop_coords, stop_id))
                @info "Trip ",tid," has out-of-bounds stop!"
                is_valid_trip = false
                break
            end

            # Calculate relative time
            stop_time = hhmmss_to_seconds(stop_row.arrival_time[1])
            @assert stop_time >= base_time
            rel_time = stop_time - base_time

            max_rel_time = (rel_time > max_rel_time) ? rel_time : max_rel_time

            trip_dict[stop_seq] = Dict("stop_id"=>stop_id, "arrival_time"=>rel_time)
        end

        if is_valid_trip
            all_trips_dict[out_trip_idx] = trip_dict
            out_trip_idx += 1
            trip_points += length(keys(trip_dict))
        end
    end

    @info "Total number of unique trips - $(out_trip_idx)"
    @info "Transit trip points - $(trip_points)"
    @info "MAx rel time - $(max_rel_time)"


    open(out_file, "w") do f
        JSON.print(f, all_trips_dict, 2)
    end
end
