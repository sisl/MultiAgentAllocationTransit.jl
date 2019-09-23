using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON
using Logging
using JLD2
using NearestNeighbors
using StaticArrays
global_logger(SimpleLogger(stderr, Logging.Error))

rng = MersenneTwister(6789)

# const city_params_file = "./data/sfmta/sf_params.toml"
# const stop_coords_file = "./data/sfmta/stop_to_coords.json"
# const trips_file = "./data/sfmta/trips.json"
# const drone_params_file = "./data/drone_params.toml"
# const bb_params_file = "./data/sfmta/sf_bb_params.toml"
# const city_travel_time_estimates = "./data/sfmta/sf_halton_tt_estimates.jld2"
# const out_file = "./data/temp_mult_generic.json"

const city_params_file = "./data/wmata/wdc_params.toml"
const bb_params_file = "./data/wmata/wdc_params.toml"
const stop_coords_file = "./data/wmata/stop_to_coords.json"
const trips_file = "./data/wmata/trips.json"
const drone_params_file = "./data/drone_params_5km.toml"
const city_travel_time_estimates = "./data/wmata/wdc_halton_tt_estimates.jld2"
const out_file = "./data/viz_soln_wdc.jld2"

# MAPF-TN params
const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.1

## Hard-code a bunch of depots and many more sites
# SF DEPOTS
# const DEPOT1 = LatLonCoords((lat = 37.762892, lon = -122.472193))
# const DEPOT2 = LatLonCoords((lat = 37.751751, lon = -122.410654))
# const DEPOT3 = LatLonCoords((lat = 37.718779, lon = -122.462401))
# depots = [DEPOT1, DEPOT2, DEPOT3]

# WDC DEPOTS
# const DEPOT1 = LatLonCoords((lat = 38.873413, lon = -77.183164))
# const DEPOT2 = LatLonCoords((lat = 38.891249, lon = -76.934652))
# const DEPOT3 = LatLonCoords((lat = 38.975575, lon = -77.030623))
# const DEPOT4 = LatLonCoords((lat = 38.796760, lon = -76.976235))
# const DEPOT5 = LatLonCoords((lat = 38.915612, lon = -77.036875))
# depots = [DEPOT1, DEPOT2, DEPOT3, DEPOT4, DEPOT5]


# Change this one
# const N_DEPOTS = length(depots)
const N_AGENTS = 200
const N_SITES = 3*N_AGENTS


city_params = parse_city_params(city_params_file)
bb_params = parse_city_params(bb_params_file)
drone_params = parse_drone_params(drone_params_file)

lat_dist = Uniform(bb_params.lat_start, bb_params.lat_end)
lon_dist = Uniform(bb_params.lon_start, bb_params.lon_end)

# Transit Graph Preprocessing
tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
tg, stop_idx_to_trips = transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)

@load city_travel_time_estimates halton_nn_tree city_halton_points travel_time_estimates

while true

    N_DEPOTS = 10
    depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]

    sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
    depot_sites = vcat(depots, sites)

    # Load OTG stuff
    otg = OffTransitGraph(depots = depots, sites = sites)
    trips_fws_dists = augmented_trip_meta_graph_fws_dists(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                                          length(depots), length(sites),
                                                          vcat(depots, sites),
                                                          drone_params)
    state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)


    # Set the cost function using the wrapper
    env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                         agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                         stop_idx_to_trips = stop_idx_to_trips, trips_fws_dists = trips_fws_dists,
                         drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                         curr_site_points = [], threshold_global_conflicts = 5)

    # cost_fn(i, j) = allocation_cost_fn_wrapper(env, ECBS_WEIGHT, N_DEPOTS, N_SITES, i, j)
    # cost_fn(i, j) = MultiAgentAllocationTransit.distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])
    cost_fn(i, j) = allocation_cost_wrapper_estimate(env, ECBS_WEIGHT, N_DEPOTS, N_SITES,
                                                    halton_nn_tree, travel_time_estimates, i, j)

    agent_tours = task_allocation(N_DEPOTS, N_SITES, N_AGENTS,
                                  depot_sites, cost_fn)
    agent_tasks = get_agent_task_set(agent_tours, N_DEPOTS, N_SITES)

    true_n_agents = length(agent_tasks)
    @show true_n_agents


    env.agent_states = [AgentState(task=agt_task) for agt_task in agent_tasks]
    env.curr_site_points = zeros(Int64, true_n_agents)
    env.plan_ref_times = zeros(true_n_agents)

    initial_states = Vector{MAPFTransitVertexState}(undef, true_n_agents)
    for i = 1:true_n_agents
     initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
    end

    solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)

    solution = PlanResult{MAPFTransitVertexState,MAPFTransitAction,Float64}[]

    try
        @time solution = search!(solver, initial_states)
    catch e
        if isa(e, DomainError)
            println("More than $(env.threshold_global_conflicts) conflicts; exiting!")
        end
        continue
    end

    initial_sites = [env.state_graph.vertices[env.depot_sites_to_vtx[string("s-", agent_tasks[i].site)]].state.location for i = 1:true_n_agents]

    set_solution_diagnostics!(env, solution)

    n_valid_firstpaths = length(env.valid_path_dists)

    if n_valid_firstpaths > 0.8 * true_n_agents
        println("Valid Solution!")
        @save out_file solution depots initial_sites
        break
    else
        println("Invalid Solution!")
    end
end

# env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
#                      agent_states = agent_states, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
#                      stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
#                      trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
#
#                      drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
#                      curr_site_points = [0 for _ = 1:true_n_agents])
# solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)
#
# @elapsed solution = search!(solver, initial_states)
