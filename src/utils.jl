## General utilities used across the board

"""
Compute the Euclidean distance between two lat-long coordinates close to each other
"""
function distance_lat_lon_euclidean(coords1::LatLongCoords, coords2::LatLongCoords)
    deglen = 110.25
    x = coords1.lat - coords2.lat
    y = (coords1.lon - coords2.lon)*cos(coords2.lat)
    return deglen*sqrt(x^2 + y^2)
end


function is_in_bounds(params::CityParams, coords::LatLongCoords)

    return (coords.lat >= params.lat_start && coords.lat <= params.lat_end) &&
            (coords.lon >= params.lon_start && coords.lon <= params.lon_end)
end
