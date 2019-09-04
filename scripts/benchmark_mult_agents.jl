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

# MAPF-TN params
const MAX_TRANSIT_CAP = 3
const ECBS_WEIGHT = 1.1
const N_DEPOT_VALS = [2, 5, 10, 15, 20]
const N_AGENT_VALS = [5, 10, 15, 20, 50, 100] # n_sites = 3 * agents

function main()

    city_params = parse_city_params(params_file)

    # TODO : Have a smaller BB on land??? (Use DREAMR data)
    lat_dist = Uniform(city_params.lat_start, city_params.lat_end)
    lon_dist = Uniform(city_params.lon_start, city_params.lon_end)

    # Transit Graph Preprocessing
    tg = load_transit_graph_latlong(stop_coords_file, trips_file, MAX_TRANSIT_CAP, rng)
    tg, stop_idx_to_trips, trips_fws_dists, stops_nn_tree, nn_idx_to_stop =
                                        transit_graph_preprocessing(tg, distance_lat_lon_euclidean)

    # Load drone params
    drone_params = parse_drone_params(drone_params_file)

    mapf_results = Dict{Int64,Dict}()

    for N_DEPOTS in N_DEPOT_VALS

        depot_results = Dict()

        for N_AGENTS in N_AGENT_VALS

            N_SITES = 3 * N_AGENTS

            # Generate depots and sites
            depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
            sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
            depot_sites = vcat(depot, sites)

            # Off transit preprocessing
            otg = OffTransitGraph(depots = depots, sites = sites)
            depot_to_sites_dists = generate_depot_to_sites_dists(otg, tg, stops_nn_tree, nn_idx_to_stop, stop_idx_to_trips,
                                                                 trips_fws_dists, dist_fn)
            state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)



            agent_tasks = task_allocation(N_DEPOTS, N_SITES, N_AGENTS,
                                          depot_sites, MultiAgentAllocationTransit.distance_lat_lon_euclidean)

            if length(agent_tasks) < N_AGENTS
                @warn "Not all agents have dpd' when depots=$(N_DEPOTS), agents=$(N_AGENTS)"
            end
            true_n_agents = length(agent_tasks)

            env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                                 agent_tasks = agent_tasks, depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                                 stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                                 trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
                                 drone_params = drone_params, dist_fn = dist_fn,
                                 curr_site_points = [0 for _ = 1:true_n_agents])

            # Setup ECBS Solver; Run search
            initial_states = Vector{MAPFTransitVertexState}(undef, true_n_agents)
            for i = 1:true_n_agents
                initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
            end

            solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)

            t = @benchmark search!(solver, initial_states)

            depot_results[true_n_agents] = Dict("mean" => mean(t.times),
                                                "median" => median(t.times),
                                                "samples" => median(t.params.samples),
                                                "std" => std(t.times))
        end

        mapf_results[N_DEPOTS] = depot_results
    end
end

# main()
