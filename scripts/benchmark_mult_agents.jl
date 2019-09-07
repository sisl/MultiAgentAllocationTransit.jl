using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON
using Logging
global_logger(SimpleLogger(stderr, Logging.Error))

rng = MersenneTwister(2345)

# Script arguments relating to transit files etc.
const city_params_file = ARGS[1]
const stop_coords_file = ARGS[2]
const trips_file = ARGS[3]
const drone_params_file = ARGS[4]
const bb_params_file = ARGS[5]
const out_file = ARGS[6]

# MAPF-TN params
const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.05
const N_DEPOT_VALS = [3, 5, 10, 15, 20]
const N_AGENT_VALS = [5, 10, 15, 20, 30, 50, 75, 100, 200] # n_sites = 3 * agents
# const N_DEPOT_VALS = [5]
# const N_AGENT_VALS = [10]
const N_TRIALS = 20

function main()

    # Load the various parameters
    city_params = parse_city_params(city_params_file)
    bb_params = parse_city_params(bb_params_file)
    drone_params = parse_drone_params(drone_params_file)

    lat_dist = Uniform(bb_params.lat_start, bb_params.lat_end)
    lon_dist = Uniform(bb_params.lon_start, bb_params.lon_end)

    # Transit Graph Preprocessing
    tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
    tg, stop_idx_to_trips, trips_fws_dists, stops_nn_tree, nn_idx_to_stop =
                    transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)


    # Setup overall results dict
    mapf_results = Dict("cap_range"=>TRANSIT_CAP_RANGE, "weight"=>ECBS_WEIGHT, "results"=>Dict())

    ## Outer loop - number of depots
    for N_DEPOTS in N_DEPOT_VALS

        # Setup results for that number of depots
        depot_results = Dict()

        # Middle loop - number of agents
        for N_AGENTS in N_AGENT_VALS

            if N_AGENTS < N_DEPOTS || N_AGENTS > N_DEPOTS*10
                continue
            end

            search_times = Float64[]
            conflicts = Int64[]
            avg_transit_options = Float64[]
            avg_path_dists = Float64[]

            # Always ignore the first trial
            for TRIAL = 1:N_TRIALS+1

                # Inner loop - number of trials
                N_SITES = 2 * N_AGENTS

                println("For $(N_DEPOTS) depots and $(N_AGENTS) agents:")

                # Generate depots and sites
                depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
                sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
                depot_sites = vcat(depots, sites)

                # Off transit preprocessing
                otg = OffTransitGraph(depots = depots, sites = sites)
                depot_to_sites_dists = generate_depot_to_sites_dists(otg, tg, stops_nn_tree, nn_idx_to_stop, stop_idx_to_trips,
                                            trips_fws_dists, MultiAgentAllocationTransit.distance_lat_lon_euclidean)
                state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)



                # Set up the shadow env to do task allocation with
                env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                                     agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                                     stops_nn_tree = stops_nn_tree, nn_idx_to_stop = nn_idx_to_stop, stop_idx_to_trips = stop_idx_to_trips,
                                     trips_fws_dists = trips_fws_dists, depot_to_sites_dists = depot_to_sites_dists,
                                     drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                     curr_site_points = [])

                cost_fn(i, j) = allocation_cost_fn_wrapper(env, ECBS_WEIGHT, N_DEPOTS, N_SITES, i, j)


                # run the task allocation, obtain the agent tasks and true number of agents
                agent_tours = task_allocation(N_DEPOTS, N_SITES, N_AGENTS,
                                              depot_sites, cost_fn)

                println("Task Allocation Done!")

                agent_tasks = get_agent_task_set(agent_tours, N_DEPOTS, N_SITES)
                true_n_agents = length(agent_tasks)

                # Now set n_agent dependent env params
                env.agent_states = [AgentState(task=agt_task) for agt_task in agent_tasks]
                env.curr_site_points = zeros(Int64, true_n_agents)
                env.plan_ref_times = zeros(true_n_agents)

                # Setup ECBS Solver; Run search
                initial_states = Vector{MAPFTransitVertexState}(undef, true_n_agents)
                for i = 1:true_n_agents
                    initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agent_tasks[i].origin)]]
                end

                solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = ECBS_WEIGHT)


                t = @elapsed solution = search!(solver, initial_states)
                println("$t seconds; $(solver.num_global_conflicts) conflicts")

                # Compute env diagnostics
                set_solution_diagnostics!(env, solution)

                push!(search_times, t)
                push!(conflicts, solver.num_global_conflicts)
                push!(avg_transit_options, mean(env.valid_transit_options))
                push!(avg_path_dists, mean(env.valid_path_dists))

            end


            depot_results[N_AGENTS] = Dict("times" => search_times[2:end],
                                            "conflicts" => conflicts[2:end],
                                            "avg_transit_options" => avg_transit_options[2:end],
                                            "avg_path_dists" => avg_path_dists[2:end])
        end
        mapf_results["results"][N_DEPOTS] = depot_results
    end

    open(out_file, "w") do f
        JSON.print(f, mapf_results, 2)
    end
end

main()
