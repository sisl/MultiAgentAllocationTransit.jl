using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using JSON

rng = MersenneTwister(2345)

sf_params_file = "../data/sfmta/sf_params.toml"
city_params = parse_city_params(sf_params_file)

out_file = "../data/allocation_timing.json"

lat_dist = Uniform(city_params.lat_start, city_params.lat_end)
lon_dist = Uniform(city_params.lon_start, city_params.lon_end)

# Fixing N_depots and N_agents = N_depots
N_DEPOTS = 10
depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
N_AGENTS = N_DEPOTS

allocation_results = Dict("n_depots"=>N_DEPOTS, "n_agents"=>N_AGENTS, "site_time"=>Dict())

N_SITE_VALS = [N for N in 20:10:100]

for N_SITES in N_SITE_VALS

    sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]

    depot_sites = vcat(depots, sites)

    cost_fn(i, j) = distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])

    site_time = @belapsed task_allocation($N_DEPOTS, $N_SITES, $N_AGENTS, $cost_fn)

    @show N_SITES
    @show site_time

    allocation_results["site_time"][N_SITES] = site_time

end

open(out_file, "w") do f
    JSON.print(f, allocation_results, 2)
end
