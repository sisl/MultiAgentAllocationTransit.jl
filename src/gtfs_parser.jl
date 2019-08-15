function generate_stop_file(stop_txt::String, params::CityParams, out_file::String)

    stop_file = CSV.File(stop_txt)

    n_stops = length(stop_file.stop_id)
    stops_dict = Dict{Int64,LatLongCoords}()

    for (i, id_str) in enumerate(stop_file.stop_id)

        id = 0
        try
            id = parse(Int64, id_str)
        catch ArgumentError
            @info "Found non-integer stop id ",id_str,", ignoring!"
            continue
        end

        @assert haskey(stops_dict, id) == false
        coords = LatLongCoords((lat = stop_file.stop_lat[i], lon = stop_file.stop_lon[i]))

        if is_in_bounds(params, coords)
            stops_dict[id] = coords
        end # if
    end

    open(out_file, "w") do stop_out_file
        JSON.print(stop_out_file, stops_dict, 2)
    end

end
