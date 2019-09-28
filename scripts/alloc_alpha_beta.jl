using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using Statistics
using JSON

## IGNORE - Just used to get some alpha beta values for supporting the theorem.

rng = MersenneTwister(2345)

const params_file = ARGS[1]
city_params = parse_city_params(params_file)

# const out_file_pref = ARGS[2]

lat_dist = Uniform(city_params.lat_start, city_params.lat_end)
lon_dist = Uniform(city_params.lon_start, city_params.lon_end)

# N_agents = N_depots
const N_DEPOTS = parse(Int64, ARGS[2])
const depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
const N_AGENTS = N_DEPOTS

# Big numbers of depots
const N_SITE_VALS = [200, 500, 1000]

for N_SITES in N_SITE_VALS

    sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]

    depot_sites = vcat(depots, sites)
    cost_fn(i, j) = MultiAgentAllocationTransit.distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])

    agent_tours, max_tour_cost, obj_value, alpha, beta = task_allocation(N_DEPOTS, N_SITES, N_AGENTS, depot_sites, cost_fn)
    n_true_agents = length(agent_tours)

    println("For $(N_SITES) sites:")
    println("Max tour cost - $(max_tour_cost); OPTLB - $(obj_value); (alpha,beta,plus) : $(alpha), $(beta), $(alpha + beta)")
    approx_factor = max_tour_cost*n_true_agents/obj_value
    println("Approx factor - $(approx_factor)")
end
