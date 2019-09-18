using Random
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Logging

global_logger(SimpleLogger(stderr, Logging.Warn))

## Hard-code a bunch of depots and many more sites
const DEPOT1 = LatLonCoords((lat = 37.762892, lon = -122.472193))
const DEPOT2 = LatLonCoords((lat = 37.751751, lon = -122.410654))
const DEPOT3 = LatLonCoords((lat = 37.718779, lon = -122.462401))

const SITE1 = LatLonCoords((lat = 37.789290, lon = -122.426797)) # Top right
const SITE2 = LatLonCoords((lat = 37.739611, lon = -122.492203)) # Way left
const SITE3 = LatLonCoords((lat = 37.780593, lon = -122.434555)) # Under SITE 1
const SITE4 = LatLonCoords((lat = 37.739011, lon = -122.430059)) # Near the bottom
const SITE5 = LatLonCoords((lat = 37.749018, lon = -122.462966)) # 2nd most left
const SITE1DUMMY = LatLonCoords((lat = 37.788042, lon = -122.429579))

const TRANSIT_CAP_RANGE = (1, 1)
const ECBS_WEIGHT = 1.05

stop_coords_file = "data/sfmta/stop_to_coords.json"
trips_file = "data/sfmta/trips.json"
drone_params_file = "data/drone_params.toml"

depots = [DEPOT1, DEPOT2, DEPOT3]
sites = [SITE1, SITE2, SITE3, SITE4, SITE5, SITE1DUMMY]

agent_tasks = [AgentTask((origin = 1, site = 1, dest = 2)),
               AgentTask((origin = 2, site = 6, dest = 1)),
               AgentTask((origin = 1, site = 3, dest = 2)),
               AgentTask((origin = 2, site = 4, dest = 3)),
               AgentTask((origin = 3, site = 5, dest = 1))
               ]

rng = MersenneTwister(1234)

# Load drone params and setup ENV!!
drone_params = parse_drone_params(drone_params_file)

# Generate OTG and load TG and drone params
otg = OffTransitGraph(depots = depots, sites = sites)
tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)


tg, stop_idx_to_trips = transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)

trips_fws_dists = augmented_trip_meta_graph_fws_dists(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                                      length(depots), length(sites),
                                                      vcat(depots, sites),
                                                      drone_params)


state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)

agent_states = [AgentState(task=agt_task) for agt_task in agent_tasks]

env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                     agent_states = agent_states, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                     stop_idx_to_trips = stop_idx_to_trips, trips_fws_dists = trips_fws_dists,
                     plan_ref_times = zeros(length(agent_tasks)),
                     drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                     curr_site_points = [0 for _ = 1:length(agent_tasks)])


# Define ECBS Solver; Run search
initial_states = Vector{MAPFTransitVertexState}(undef, length(agent_tasks))
for i = 1:length(agent_tasks)
    initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
end

solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)

solution = search!(solver, initial_states)
#
# # NEED TO RE_INIT SOLVER!!!!
agent_states = [AgentState(task=agt_task) for agt_task in agent_tasks]

env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                     agent_states = agent_states, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                     stop_idx_to_trips = stop_idx_to_trips, trips_fws_dists = trips_fws_dists,
                     plan_ref_times = zeros(length(agent_tasks)), valid_transit_options = zeros(length(agent_tasks)),
                     drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                     curr_site_points = [0 for _ = 1:length(agent_tasks)])


solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)
@time solution = search!(solver, initial_states)
