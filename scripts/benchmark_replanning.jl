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

rng = MersenneTwister(2345)

# Script arguments relating to transit files etc.
const city_params_file = ARGS[1]
const stop_coords_file = ARGS[2]
const trips_file = ARGS[3]
const drone_params_file = ARGS[4]
const bb_params_file = ARGS[5]
const out_file_pref = ARGS[6]
const city_travel_time_estimates = ARGS[7]
const N_DEPOTS = parse(Int64, ARGS[8])
const N_AGENTS = parse(Int64, ARGS[9])
const N_TRIALS = parse(Int64, ARGS[10])

const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.05


function main()

    # Load the various parameters
    city_params = parse_city_params(city_params_file)
    bb_params = parse_city_params(bb_params_file)
    drone_params = parse_drone_params(drone_params_file)

    lat_dist = Uniform(bb_params.lat_start, bb_params.lat_end)
    lon_dist = Uniform(bb_params.lon_start, bb_params.lon_end)

    # Transit Graph Preprocessing
    tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
    tg, stop_idx_to_trips = transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)


    # Load Halton stuff
    @load city_travel_time_estimates halton_nn_tree city_halton_points travel_time_estimates

    replan_results = Dict("cap_range"=>TRANSIT_CAP_RANGE, "weight"=>ECBS_WEIGHT,
                        "depots"=>N_DEPOTS, "agents"=>N_AGENTS)

    rp_one_times = Float64[]
    rp_one_msps = Float64[]
    rp_all_times = Float64[]
    rp_all_msps = Float64[]

    # No need to ignore first trial now!
    TRIAL = 1
    while TRIAL <= N_TRIALS+1

        N_SITES = 2*N_AGENTS

        println("Trial $(TRIAL)")

        # Generate depots and sites
        depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
        sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
        depot_sites = vcat(depots, sites)

        # Off transit preprocessing
        otg = OffTransitGraph(depots = depots, sites = sites)
        trips_fws_dists = augmented_trip_meta_graph_fws_dists(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                                              length(depots), length(sites),
                                                              vcat(depots, sites),
                                                              drone_params)
        state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)



        # Set up the shadow env to do task allocation with
        env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                             agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                             stop_idx_to_trips = stop_idx_to_trips, trips_fws_dists = trips_fws_dists,
                             drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                             curr_site_points = [], threshold_global_conflicts = 10)

        # run the task allocation, obtain the agent tasks and true number of agents
        cost_fn(i, j) = allocation_cost_wrapper_estimate(env, ECBS_WEIGHT, N_DEPOTS, N_SITES,
                                                        halton_nn_tree, travel_time_estimates, i, j)
        agent_tours = task_allocation(N_DEPOTS, N_SITES, N_AGENTS,
                                      depot_sites, cost_fn)

        println("Task Allocation Done!")
        @show agent_tours


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


        solution = search!(solver, initial_states)

        println("Initial solution obtained!")

        # Compute env diagnostics and throw away if invalid
        set_solution_diagnostics!(env, solution)
        n_valid_firstpaths = length(env.valid_path_dists)

        if n_valid_firstpaths > 0.67 * true_n_agents

            # Copy env and solution for replanning
            env_copy = deepcopy(env)
            solution_copy = deepcopy(solution)

            # Actually run the replanning
            did_replan_indiv, el_time_indiv = replan_individual!(env, solution, N_DEPOTS, N_SITES, agent_tours, ECBS_WEIGHT)
            did_replan_collec, el_time_collec, new_soln_collec = replan_collective!(env_copy, solution_copy, N_DEPOTS, N_SITES, agent_tours, ECBS_WEIGHT)

            # Enter only if both valid
            if did_replan_indiv && did_replan_collec

                println("Replanning done successfully!")

                # Now look at times and makespans
                msp_one = maximum([s.cost for s in solution])
                msp_all = maximum([s.cost for s in new_soln_collec])

                push!(rp_one_times, el_time_indiv)
                push!(rp_one_msps, msp_one)

                push!(rp_all_times, el_time_collec)
                push!(rp_all_msps, msp_all)

                # Increment trial
                TRIAL = TRIAL + 1

            else

                println("No replan! Indiv - $(did_replan_indiv); Collec - $(did_replan_collec)")

            end
        end
    end

    replan_results["results"] = Dict("replan_one_times" => rp_one_times,
                                     "replan_one_makespans" => rp_one_msps,
                                     "replan_all_times" => rp_all_times,
                                     "replan_all_makespans" => rp_all_msps)

    out_file = string(out_file_pref, "_", N_DEPOTS, "deps_", N_AGENTS, "_agts.json")
    open(out_file, "w") do f
        JSON.print(f, replan_results, 2)
    end
end

main()
