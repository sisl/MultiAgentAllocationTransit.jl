using Random
using MultiAgentPathFinding
using MultiAgentAllocationTransit


## Toy example for one agent (depot1 -> site -> depot2) in SF
const DEPOT_ORIG = LatLonCoords((lat = 37.762892, lon = -122.472193))
const SITE = LatLonCoords((lat = 37.789290, lon = -122.426797))
const DEPOT_DEST = LatLonCoords((lat = 37.751751, lon = -122.410654))
const MAX_TRANSIT_CAP = 1
WEIGHT = 1.05

stop_coords_file = "data/sfmta/stop_to_coords.json"
trips_file = "data/sfmta/trips.json"
drone_params_file = "data/drone_params.toml"

# function main(stop_coords_file::String, trips_file::String)

# Hard-code the depot and site locations - get agent_tasks
depots = [DEPOT_ORIG, DEPOT_DEST]
sites = [SITE]
task = AgentTask((origin = 1, site = 1, dest = 2))
agent_tasks = [task]

rng = MersenneTwister(1234)

# Load drone params and setup ENV!!
drone_params = parse_drone_params(drone_params_file)

# Generate OTG and load TG and drone params
otg = OffTransitGraph(depots = depots, sites = sites)
tg = load_transit_graph_latlong(stop_coords_file, trips_file, MAX_TRANSIT_CAP, rng)


# Do preprocessing of TG to get NN stuff and TIRG stuff and depot_to_site stuff
stop_idx_to_trips = get_stop_idx_to_trip_ids(tg)

# Can possibly omit this
true_stop_to_locs = true_stop_to_locations(tg.stop_to_location, stop_idx_to_trips)
tg = TransitGraph(true_stop_to_locs, tg.transit_trips, tg.transit_capacity)

trips_fws_dists = trip_meta_graph_fws_dists(tg, distance_lat_lon_euclidean, drone_params)
# stops_nn_tree, nn_idx_to_stop = stop_locations_nearest_neighbors(tg.stop_to_location, EuclideanLatLong())
# depot_to_sites_dists = generate_depot_to_sites_dists(otg, tg, stops_nn_tree, nn_idx_to_stop, stop_idx_to_trips,
#                                                      trips_fws_dists, dist_fn)
#
# # Generate state_graph, depot_sites_to_vtx, and trip_to_vtx_range
# state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)
#
#
#
# env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
#                      agent_tasks = agent_tasks, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
#                      stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
#                      trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
#                      drone_params = drone_params, dist_fn = dist_fn)
#
# # Define ECBS Solver; Run search
# initial_states = Vector{MAPFTransitVertexState}(undef, 0)
# push!(initial_states, MAPFTransitVertexState(0, MAPFTransitState(0.0, LatLonCoords()), ""))
#
# solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = WEIGHT)

# end
