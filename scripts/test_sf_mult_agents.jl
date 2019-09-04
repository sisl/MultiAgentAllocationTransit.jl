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

const MAX_TRANSIT_CAP = 1
WEIGHT = 1.05

stop_coords_file = "data/sfmta/stop_to_coords.json"
trips_file = "data/sfmta/trips.json"
drone_params_file = "data/drone_params.toml"

depots = [DEPOT1, DEPOT2, DEPOT3]
sites = [SITE1, SITE2, SITE3, SITE4, SITE5, SITE1DUMMY]

agent_tasks = [AgentTask((origin = 1, site = 1, dest = 2)),
               AgentTask((origin = 1, site = 2, dest = 3)),
               AgentTask((origin = 1, site = 3, dest = 2)),
               AgentTask((origin = 2, site = 4, dest = 3)),
               AgentTask((origin = 3, site = 5, dest = 1))
               ]

rng = MersenneTwister(1234)

# Load drone params and setup ENV!!
drone_params = parse_drone_params(drone_params_file)

# Generate OTG and load TG and drone params
otg = OffTransitGraph(depots = depots, sites = sites)
tg = load_transit_graph_latlong(stop_coords_file, trips_file, MAX_TRANSIT_CAP, rng)

# Do preprocessing of TG to get NN stuff and TIRG stuff and depot_to_site stuff
# stop_idx_to_trips = get_stop_idx_to_trip_ids(tg)
#
# # Can possibly omit this
# true_stop_to_locs = true_stop_to_locations(tg.stop_to_location, stop_idx_to_trips)
# tg = TransitGraph(true_stop_to_locs, tg.transit_trips, tg.transit_capacity)
#
# trips_fws_dists = trip_meta_graph_fws_dists(tg, dist_fn)
# stops_nn_tree, nn_idx_to_stop = stop_locations_nearest_neighbors(tg.stop_to_location, EuclideanLatLong())

tg, stop_idx_to_trips, trips_fws_dists, stops_nn_tree, nn_idx_to_stop = transit_graph_preprocessing(tg, distance_lat_lon_euclidean, drone_params)


depot_to_sites_dists = generate_depot_to_sites_dists(otg, tg, stops_nn_tree, nn_idx_to_stop, stop_idx_to_trips,
                                                     trips_fws_dists, distance_lat_lon_euclidean)

# Generate state_graph, depot_sites_to_vtx, and trip_to_vtx_range
state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)


env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                     agent_tasks = agent_tasks, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                     stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                     trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
                     drone_params = drone_params, dist_fn = distance_lat_lon_euclidean,
                     curr_site_points = [0 for _ = 1:length(agent_tasks)])


# Define ECBS Solver; Run search
initial_states = Vector{MAPFTransitVertexState}(undef, length(agent_tasks))
for i = 1:length(agent_tasks)
    initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
end

solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = WEIGHT)

# Setup functions for pre-processing etc.
# @btime search!($solver, $initial_states)
search!(solver, initial_states)
@time solution = search!(solver, initial_states)
