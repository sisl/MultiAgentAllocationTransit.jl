using Random
using Distributions
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JLD2
using Logging
global_logger(SimpleLogger(stderr, Logging.Error))

rng = MersenneTwister(6789)

# EXAMPLES OF ARGUMENTS
# const bb_params_file = "./data/sfmta/sf_bb_params.toml"
# const stop_coords_file = "./data/sfmta/stop_to_coords.json"
# const trips_file = "./data/sfmta/trips.json"
# const drone_params_file = "./data/drone_params.toml"
# const city_tt_outfile = "./data/sfmta/sf_halton_travel_estimates.jld2"
# const n_halton_points = 100

const bb_params_file = ARGS[1]
const stop_coords_file = ARGS[2]
const trips_file = ARGS[3]
const drone_params_file = ARGS[4]
const city_tt_outfile = ARGS[5]
const n_halton_points = parse(Int64, ARGS[6])

const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.1



function main()
    bb_params = parse_city_params(bb_params_file)
    drone_params = parse_drone_params(drone_params_file)

    tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
    tg, stop_idx_to_trips, aug_trips_fws_dists, stops_nn_tree, nn_idx_to_stop =
                    transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)


    # Generate halton NN of city
    halton_nn_tree, city_halton_points = generate_city_halton_nn(bb_params; n_points = n_halton_points)

    # Now create a bunch of fake depots and sites
    midway = n_halton_points / 2
    depots = [LatLonCoords((lat = chp[1], lon = chp[2])) for chp in city_halton_points]
    # sites = [LatLonCoords((lat = chp[1], lon = chp[2])) for chp in city_halton_points[midway+1:end]]


    # Create OTG
    dummy_otg = OffTransitGraph(depots = depots, sites = LatLonCoords[])
    state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, dummy_otg)

    env = MAPFTransitEnv(off_transit_graph = dummy_otg, transit_graph = tg, state_graph = state_graph,
                         agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                         stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                         aug_trips_fws_dists = aug_trips_fws_dists, depot_to_sites_dists = Matrix{Float64}(undef, 0, 0),
                         drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                         curr_site_points = [], threshold_global_conflicts = 0)

    println("Computing estimates")
    @time travel_time_estimates = compute_all_pairs_estimates(env, n_halton_points, ECBS_WEIGHT)

    # Save to file
    @save city_tt_outfile halton_nn_tree city_halton_points travel_time_estimates
end

main()
