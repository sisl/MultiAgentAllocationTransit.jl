using Random
using MultiAgentPathFinding
using MultiAgentAllocationTransit

## Hard-code a bunch of depots and many more sites
const DEPOT1 = LatLonCoords((lat = 37.762892, lon = -122.472193))
const DEPOT2 = LatLonCoords((lat = 37.751751, lon = -122.410654))
const DEPOT3 = LatLonCoords((lat = 37.718779, lon = -122.462401))

const SITE1 = LatLonCoords((lat = 37.789290, lon = -122.426797)) # Top right
const SITE2 = LatLonCoords((lat = 37.739611, lon = -122.492203)) # Way left
const SITE3 = LatLonCoords((lat = 37.780593, lon = -122.434555)) # Under SITE 1
const SITE4 = LatLonCoords((lat = 37.739011, lon = -122.430059)) # Near the bottom
const SITE5 = LatLonCoords((lat = 37.749018, lon = -122.462966)) # 2nd most left

depots = [DEPOT1, DEPOT2, DEPOT3]
sites = [SITE1, SITE2, SITE3, SITE4, SITE5]
depot_sites = vcat(depots, sites)

n_depots = 3
n_agents = 5
n_sites = 5


# # Run the task allocation
# edges, x_edges, vds, dsv, cv = min_connecting_tour(n_depots, n_sites, cost_fn)
#
# # Get connected depots
# depot_comps = get_connected_depots(n_depots, n_sites, edges)
#
# # Merge the depots and add edges
# add_merged_depot_edges!(x_edges, depot_comps, dsv, n_depots, n_sites, cv)
#
# # Get Eulerian circuit
# circuit = get_multiedge_eulerian_tour(x_edges, vds, n_depots, n_sites, cost_fn)
cost_fn(i, j) = MultiAgentAllocationTransit.distance_lat_lon_euclidean(depot_sites[i], depot_sites[j])
agent_tours = task_allocation(n_depots, n_sites, n_agents, depot_sites, cost_fn)

# agent_tasks = get_agent_task_set(agent_tours, n_depots, n_sites)
