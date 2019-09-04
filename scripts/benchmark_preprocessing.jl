using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON
using Logging
global_logger(SimpleLogger(stderr, Logging.Warn))

rng = MersenneTwister(2345)

# Script arguments relating to transit files etc.
const city_params_file = ARGS[1]
const stop_coords_file = ARGS[2]
const trips_file = ARGS[3]
const drone_params_file = ARGS[4]
const out_file = ARGS[5]

const MAX_TRANSIT_CAP = 3

tg = load_transit_graph_latlong(stop_coords_file, trips_file, MAX_TRANSIT_CAP, rng)
drone_params = parse_drone_params(drone_params_file)

t = @benchmark transit_graph_preprocessing($tg, distance_lat_lon_euclidean, $drone_params)
pp_dict = Dict("city"=>city_params_file,
               "mean"=>mean(t.times),
               "median"=>median(t.times))

open(out_file, "w") do f
    JSON.print(f, pp_dict, 2)
end
