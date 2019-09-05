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
const city_params_file = "./data/sfmta/sf_params.toml"
const stop_coords_file = "./data/sfmta/stop_to_coords.json"
const trips_file = "./data/sfmta/trips.json"
const drone_params_file = "./data/drone_params.toml"
const bb_params_file = "./data/sfmta/sf_bb_params.toml"
const out_file = "./data/temp_mult_generic.json"

# MAPF-TN params
const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.05
const N_DEPOTS = 3

# Change this one
const N_AGENTS = 10

const N_SITES = 2*N_AGENTS

## Hard-code a bunch of depots and many more sites
const DEPOT1 = LatLonCoords((lat = 37.762892, lon = -122.472193))
const DEPOT2 = LatLonCoords((lat = 37.751751, lon = -122.410654))
const DEPOT3 = LatLonCoords((lat = 37.718779, lon = -122.462401))
depots = [DEPOT1, DEPOT2, DEPOT3]

city_params = parse_city_params(city_params_file)
bb_params = parse_city_params(bb_params_file)
drone_params = parse_drone_params(drone_params_file)

lat_dist = Uniform(bb_params.lat_start, bb_params.lat_end)
lon_dist = Uniform(bb_params.lon_start, bb_params.lon_end)

# Transit Graph Preprocessing
tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
tg, stop_idx_to_trips, trips_fws_dists, stops_nn_tree, nn_idx_to_stop =
                transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)

sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
depot_sites = vcat(depots, sites)

# Load OTG stuff
otg = OffTransitGraph(depots = depots, sites = sites)
depot_to_sites_dists = generate_depot_to_sites_dists(otg, tg, stops_nn_tree, nn_idx_to_stop, stop_idx_to_trips,
                            trips_fws_dists, MultiAgentAllocationTransit.distance_lat_lon_euclidean)
state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)


# Set the cost function using the wrapper
env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                     agent_tasks = AgentTask[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                     stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                     trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
                     drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                     curr_site_points = [])

cost_fn(i, j) = allocation_cost_fn_wrapper(env, ECBS_WEIGHT, N_DEPOTS, N_SITES, i, j)


agent_tours = task_allocation(N_DEPOTS, N_SITES, N_AGENTS,
                              depot_sites, cost_fn)
agent_tasks = get_agent_task_set(agent_tours, N_DEPOTS, N_SITES)

true_n_agents = length(agent_tasks)
@show true_n_agents


env.agent_tasks = agent_tasks
env.curr_site_points = [0 for _ = 1:true_n_agents]

initial_states = Vector{MAPFTransitVertexState}(undef, true_n_agents)
for i = 1:true_n_agents
 initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
end

solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)

search!(solver, initial_states)

env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                     agent_tasks = agent_tasks, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                     stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                     trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
                     drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                     curr_site_points = [0 for _ = 1:true_n_agents])
solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)

@elapsed solution = search!(solver, initial_states)
