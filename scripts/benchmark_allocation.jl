using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON

rng = MersenneTwister(2345)

params_file = ARGS[1]
city_params = parse_city_params(params_file)

out_file = ARGS[2]

lat_dist = Uniform(city_params.lat_start, city_params.lat_end)
lon_dist = Uniform(city_params.lon_start, city_params.lon_end)

# Fixing N_depots and N_agents = N_depots
N_DEPOTS = 10
depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
N_AGENTS = N_DEPOTS

allocation_results = Dict("n_depots"=>N_DEPOTS, "n_agents"=>N_AGENTS, "site_stats"=>Dict())


N_SITE_VALS = [N for N in 20:10:100]

for N_SITES in N_SITE_VALS

    sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]

    depot_sites = vcat(depots, sites)

    b = @benchmarkable task_allocation($N_DEPOTS, $N_SITES, $N_AGENTS,
        depot_sites, MultiAgentAllocationTransit.distance_lat_lon_euclidean)  setup=(depot_sites = vcat($depots, [LatLonCoords((lat = rand($rng, $lat_dist), lon = rand($rng, $lon_dist))) for i = 1:$N_SITES]))

    t = run(b) # Do once to trigger...something
    t = run(b)

    @show N_SITES
    @show mean(t.times)

    allocation_results["site_stats"][N_SITES] = Dict("mean" => mean(t.times),
                                                     "median" => median(t.times),
                                                     "std" => std(t.times),
                                                     "samples" => t.params.samples)

end

open(out_file, "w") do f
    JSON.print(f, allocation_results, 2)
end
