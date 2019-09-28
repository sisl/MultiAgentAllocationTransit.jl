using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON
using Logging
global_logger(SimpleLogger(stderr, Logging.Error))

rng = MersenneTwister(6789)

# Script arguments relating to transit files etc.
const city_params_file = ARGS[1]
const stop_coords_file = ARGS[2]
const trips_file = ARGS[3]
const drone_params_file = ARGS[4]
const bb_params_file = ARGS[5]
const out_file_pref = ARGS[6]
const N_DEPOTS = parse(Int64, ARGS[7])
const N_AGENTS = parse(Int64, ARGS[8])
const N_TRIALS = parse(Int64, ARGS[9])

# MAPF-TN params
const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.1

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


    mapf_results = Dict("cap_range"=>TRANSIT_CAP_RANGE, "weight"=>ECBS_WEIGHT,
                        "depots"=>N_DEPOTS, "agents"=>N_AGENTS)

    search_times = Float64[]
    conflicts = Int64[]
    max_transit_options = Float64[]
    max_path_dists = Float64[]
    makespans = Float64[]

    # Always ignore the first trial
    TRIAL = 1
    while TRIAL <= N_TRIALS+1

        # Inner loop - number of trials
        N_SITES = 2 * N_AGENTS

        println("Trial $(TRIAL)")

        # Generate depots and sites
        depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
        sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
        depot_sites = vcat(depots, sites)

        # Off transit preprocessing
        otg = OffTransitGraph(depots = depots, sites = sites)

        aug_trips_fws_dists = augmented_trip_meta_graph_fws_dists(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                                              length(depots), length(sites),
                                                              vcat(depots, sites),
                                                              drone_params)

        state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)



        # Set up the shadow env to do task allocation with
        env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                             agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                             stop_idx_to_trips = stop_idx_to_trips, aug_trips_fws_dists = aug_trips_fws_dists,
                             drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                             curr_site_points = [], threshold_global_conflicts = 5)

        # run the task allocation, obtain the agent tasks and true number of agents
        cost_fn(i, j) = MultiAgentAllocationTransit.distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])


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

        # Try catch for conflicts
        t = 0.0
        solution = PlanResult{MAPFTransitVertexState,MAPFTransitAction,Float64}[]
        try
            t = @elapsed solution = search!(solver, initial_states)
        catch e
            if isa(e, DomainError)
                println("More than $(env.threshold_global_conflicts) conflicts; exiting!")
            end
            continue
        end

        println("$t seconds; $(solver.num_global_conflicts) conflicts")

        # Compute env diagnostics
        set_solution_diagnostics!(env, solution)

        n_valid_firstpaths = length(env.valid_path_dists)

        if n_valid_firstpaths > 0.67 * true_n_agents

            msp = maximum([s.cost for s in solution])

            push!(search_times, t)
            push!(conflicts, solver.num_global_conflicts)
            push!(max_transit_options, maximum(env.valid_transit_options))
            push!(max_path_dists, maximum(env.valid_path_dists))
            push!(makespans, msp)

            TRIAL = TRIAL + 1
        end
    end


    mapf_results["results"] =   Dict("times" => search_times[2:end],
                                    "conflicts" => conflicts[2:end],
                                    "max_transit_options" => max_transit_options[2:end],
                                    "max_path_dists" => max_path_dists[2:end],
                                    "makespans"=>makespans)

    out_file_name = string(out_file_pref,"_",N_DEPOTS,"deps_",N_AGENTS,"_agts.json")
    open(out_file_name, "w") do f
        JSON.print(f, mapf_results, 2)
    end
end

main()
