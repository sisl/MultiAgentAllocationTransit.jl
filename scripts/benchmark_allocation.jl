using Random
using Distributions
using MultiAgentPathFinding
using MultiAgentAllocationTransit
using BenchmarkTools
using Statistics
using JSON

rng = MersenneTwister(2345)

const params_file = ARGS[1]
city_params = parse_city_params(params_file)

const out_file_pref = ARGS[2]

lat_dist = Uniform(city_params.lat_start, city_params.lat_end)
lon_dist = Uniform(city_params.lon_start, city_params.lon_end)

# N_agents = N_depots
const N_DEPOTS = parse(Int64, ARGS[3])
const depots = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_DEPOTS]
const N_AGENTS = N_DEPOTS

const N_SITE_VALS = [20, 50, 100, 200, 500, 1000, 5000]

for N_SITES in N_SITE_VALS

    sites = [LatLonCoords((lat = rand(rng, lat_dist), lon = rand(rng, lon_dist))) for i = 1:N_SITES]

    depot_sites = vcat(depots, sites)
    cost_fn(i, j) = MultiAgentAllocationTransit.distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])

    b = @benchmarkable task_allocation($N_DEPOTS, $N_SITES, $N_AGENTS,
        depot_sites, $cost_fn)  setup=(depot_sites = vcat($depots, [LatLonCoords((lat = rand($rng, $lat_dist), lon = rand($rng, $lon_dist))) for i = 1:$N_SITES]))

    t = run(b) # Do once to trigger...something
    t = run(b)

    @show N_SITES
    @show mean(t.times)

    allocation_results = Dict("n_depots"=>N_DEPOTS, "n_agents"=>N_AGENTS, "site_stats"=>Dict())
    allocation_results["site_stats"][N_SITES] = Dict("mean" => mean(t.times),
                                                     "median" => median(t.times),
                                                     "std" => std(t.times),
                                                     "samples" => t.params.samples)

    out_file = string(out_file_pref,"_",N_SITES,"sites.json")
    open(out_file, "w") do f
        JSON.print(f, allocation_results, 2)
    end
end
