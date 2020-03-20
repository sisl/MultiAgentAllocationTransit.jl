using Random
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using Infiltrator
using JLD2
using Distributions
using NearestNeighbors
using JSON
using Statistics
using StaticArrays

## Hard-code the three depots of interest; centrally located
const DEPOT1 = LatLonCoords((lat = 37.762892, lon = -122.472193))
const DEPOT2 = LatLonCoords((lat = 37.751751, lon = -122.410654))
const DEPOT3 = LatLonCoords((lat = 37.718779, lon = -122.462401))
const N_DEPOTS = 3
const N_AGENTS = 15
const N_SITES = 80

# Hard code arguments for now
const bb_params_file = "./data/sfmta/sf_bb_params.toml"
const stop_coords_file = "./data/sfmta/stop_to_coords.json"
const trips_file = "./data/sfmta/trips.json"
const drone_params_file = "./data/drone_params.toml"
const city_travel_time_estimates = "./data/sfmta/sf_halton_tt_estimates.jld2"
const zipcode_file = "./data/sf-zip-code-latitude-and-longitude.json"
const zipcode_income_file = "./data/sfmta/sf_median_income.toml"

# MAPF-TN params
const TRANSIT_CAP_RANGE = (3, 5)
const ECBS_WEIGHT = 1.1

function get_zc_seed(by_income::Bool, rng::RNG=Random.GLOBAL_RNG) where {RNG <: AbstractRNG}

    bb_params = parse_city_params(bb_params_file)
    drone_params = parse_drone_params(drone_params_file)

    lat_dist = Uniform(bb_params.lat_start, bb_params.lat_end)
    lon_dist = Uniform(bb_params.lon_start, bb_params.lon_end)

    tg = load_transit_graph_latlong(stop_coords_file, trips_file, TRANSIT_CAP_RANGE, rng)
    tg, stop_idx_to_trips = transit_graph_preprocessing(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean, drone_params)

    @load city_travel_time_estimates halton_nn_tree city_halton_points travel_time_estimates


    # Get zipcode NN info for sampling sites
    ztree, zstrs = construct_zipcode_nn(zipcode_file)

    zipcode_counts = Dict{String,Int64}()
    for z in zstrs
        zipcode_counts[z] = 0
    end

    # Generate sites based on income or uniform and concat with known depots
    depots = [DEPOT1, DEPOT2, DEPOT3]

    if by_income == true
        sites = generate_sites_by_income(ztree, zstrs, zipcode_income_file, N_SITES, lat_dist, lon_dist, rng)
    else
        sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]
    end

    depot_sites = vcat(depots, sites)

    # Off transit preprocessing
    otg = OffTransitGraph(depots = depots, sites = sites)
    aug_trips_fws_dists = augmented_trip_meta_graph_fws_dists(tg, MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                                                          length(depots), length(sites),
                                                          vcat(depots, sites),
                                                          drone_params)
    state_graph, depot_sites_to_vtx, trip_to_vtx_range = setup_state_graph(tg, otg)



    env = MAPFTransitEnv(off_transit_graph = otg, transit_graph = tg, state_graph = state_graph,
                         agent_states = AgentState[], depot_sites_to_vtx = depot_sites_to_vtx, trip_to_vtx_range = trip_to_vtx_range,
                         stop_idx_to_trips = stop_idx_to_trips, aug_trips_fws_dists = aug_trips_fws_dists,
                         drone_params = drone_params, dist_fn = MultiAgentAllocationTransit.distance_lat_lon_euclidean,
                         curr_site_points = [], threshold_global_conflicts = 10)

    # run the task allocation, obtain the agent tasks and true number of agents
    # cost_fn(i, j) = allocation_cost_fn_wrapper_truett(env, ECBS_WEIGHT, N_DEPOTS, N_SITES, i, j)
    cost_fn(i, j) = allocation_cost_wrapper_estimate(env, ECBS_WEIGHT, N_DEPOTS, N_SITES,
                                                    halton_nn_tree, travel_time_estimates, i, j)

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

    solution = PlanResult{MAPFTransitVertexState,MAPFTransitAction,Float64}[]
    solution = search!(solver, initial_states)

    set_solution_diagnostics!(env, solution)
    n_valid_firstpaths = length(env.valid_path_dists)

    if n_valid_firstpaths > 0.67 * true_n_agents
        # Iterate over PlanResult members of solution
        # NOTE: No notion of time spent
        for pr in solution
            for (mapftvs, _) in pr.states
                loc_vect = [mapftvs.state.location.lat, mapftvs.state.location.lon]
                idxs, _ = knn(ztree, loc_vect, 1)
                zip = zstrs[idxs[1]]
                zipcode_counts[zip] += 1
            end
        end # pr in solution
    end

    return zipcode_counts
end # function get_zc_seed

function main(outfile::String, by_income::Bool, trials::Int64=10, base_seed::Int64=1234)

    global_zcounts = Dict{String,Vector{Int64}}()

    for i = 1:trials
        zcounts = get_zc_seed(by_income, MersenneTwister(base_seed*i))

        for (zip, count) in zcounts
            if count > 0
                if haskey(global_zcounts, zip)
                    push!(global_zcounts[zip], count)
                else
                    global_zcounts[zip] = [count]
                end
            end
        end
    end

    # post processing to actually get means
    # Zip code to mean, std
    zipcode_stats = Dict{String,SVector{2,Float64}}()
    for (zc, counts) in global_zcounts
        padded_counts = convert(Vector{Int64}, counts)
        padreq = trials - length(padded_counts)
        for _ = 1:padreq
            push!(padded_counts, 0)
        end
        zipcode_stats[zc] = @SVector [mean(padded_counts), std(padded_counts)]
    end

    res_dict = Dict("trials"=>trials, "zipcode_counts"=>global_zcounts,
                    "zipcode_stats"=>zipcode_stats)

    open(outfile, "w") do f
        JSON.print(f, res_dict, 2)
    end
end
