# cost_fn returns infinite for excluded edges
# Excluded edges -
function min_connecting_tour(n_depots::Int64, n_sites::Int64,
                             depot_sites::Vector{LOC},
                             cost_fn::F) where {LOC, F <: Function}


    # Construct the cost vector (no inf terms!)
    # cost_fn takes as argument u_idx and v_idx (1...n_depots....n_depots + n_sites)
    cost_vector = Vector{Float64}(undef, 0)
    depot_site_vector_idx = Dict{Tuple{Int64,Int64},Int64}()
    vector_idx_to_depot_site = Dict{Int64,Tuple{Int64,Int64}}()
    out_nbrs = Dict{Int64,Set{Int64}}()
    in_nbrs = Dict{Int64,Set{Int64}}()

    for i = 1:n_depots+n_sites
        for j = 1:n_depots+n_sites

            # Exclude self-edges and site-site edges
            if i != j && (i <= n_depots || j <= n_depots)
                edge_cost = cost_fn(depot_sites[i], depot_sites[j])
                if edge_cost < Inf
                    # First insert cost and idx
                    push!(cost_vector, edge_cost)
                    depot_site_vector_idx[(i, j)] = length(cost_vector)
                    vector_idx_to_depot_site[length(cost_vector)] = (i, j)

                    if ~(haskey(out_nbrs, i))
                        out_nbrs[i] = Set{Int64}()
                    end
                    push!(out_nbrs[i], j)

                    if ~(haskey(in_nbrs, j))
                        in_nbrs[j] = Set{Int64}()
                    end
                    push!(in_nbrs[j], i)
                end
            end
        end
    end

    # @show depot_site_vector_idx
    # @show vector_idx_to_depot_site

    # Since x vector will be subset of (l+k) x (l+k), need to maintain a map
    # from (l,k) to index and index to (l,k)
    model = Model(with_optimizer(GLPK.Optimizer))
    n_idxs = length(cost_vector)

    @variable(model, x[1:n_idxs]) # True for all edges
    @constraint(model, x .>= 0)
    @objective(model, Min, cost_vector' * x)

    # Add {0,1} constraint on package-depot / depot-package edges
    depot_site_mask = zeros(n_depots*n_sites*2, n_idxs)
    for i = 1:n_depots # Depots
        for j = n_depots+1:n_depots+n_sites # Sites
            # Depot -> site
            idx = (i - 1)*n_sites + (j - n_depots)

            # @show (i, j, idx, idx + n_depots*n_sites)

            if haskey(depot_site_vector_idx, (i, j))
                depot_site_mask[idx, depot_site_vector_idx[(i, j)]] = 1.0
            end

            # Site -> Depot
            if haskey(depot_site_vector_idx, (j, i))
                depot_site_mask[idx + n_depots*n_sites, depot_site_vector_idx[(j, i)]] = 1.0
            end
        end
    end
    # @show depot_site_mask
    @constraint(model, depot_site_mask * x .<= ones(n_depots*n_sites*2)) # Depot<->site edges at most once

    # ALL OTHERS ARE DEPOT-DEPOT right? YES!

    # Now add constraints for out-edges and in-edges for sites
    site_out_edge_mask = zeros(n_sites, n_idxs)
    site_in_edge_mask = zeros(n_sites, n_idxs)
    for i = n_depots+1:n_depots+n_sites

        if isempty(out_nbrs[i])
            @warn "Out nbrs for site $i is empty!"
            readline()
        end
        for onbr in out_nbrs[i] # Should only be depots
            idx = depot_site_vector_idx[(i, onbr)]
            site_out_edge_mask[i - n_depots, idx] = 1.
        end

        if isempty(in_nbrs[i])
            @warn "In nbrs for site $i is empty!"
            readline()
        end
        for inbr in in_nbrs[i]
            idx = depot_site_vector_idx[(inbr, i)]
            site_in_edge_mask[i - n_depots, idx] = 1.
        end
    end

    @constraint(model, site_out_edge_mask * x .== ones(n_sites))
    @constraint(model, site_in_edge_mask * x .== ones(n_sites))


    # Finally, add constraints for in-flow and out-flow from depots
    depot_out_edge_mask = zeros(n_depots, n_idxs)
    depot_in_edge_mask = zeros(n_depots, n_idxs)
    for i = 1 : n_depots

        for onbr in out_nbrs[i]
            idx = depot_site_vector_idx[(i, onbr)]
            depot_out_edge_mask[i, idx] = 1.
        end

        for inbr in in_nbrs[i]
            idx = depot_site_vector_idx[(inbr, i)]
            depot_in_edge_mask[i, idx] = 1
        end

        # @constraint(model, depot_out_edge_mask[i, :]' * x - depot_in_edge_mask[i, :]' *x == 0)
    end

    @constraint(model, depot_out_edge_mask * x - depot_in_edge_mask * x .== zeros(n_depots))

    # Now we optimize!
    JuMP.optimize!(model)

    x_edges = JuMP.value.(x)

    # Integrality gap 0
    x_edges = convert(Vector{Int64}, x_edges)

    edges = [vector_idx_to_depot_site[i] for (i, val) in enumerate(x_edges) if val > 0] # not just = 1 as may have > 1

    return (edges, x_edges, vector_idx_to_depot_site, depot_site_vector_idx, cost_vector)

end # function


# idxs are always 1....n_depots....n_depots+n_sites
function get_connected_depots(n_depots::Int64, n_sites::Int64, edges::Vector{Tuple{Int64,Int64}})

    adj_mat = zeros(Int64, n_depots+n_sites, n_depots+n_sites)

    for t in edges
        adj_mat[t[1], t[2]] = 1
    end

    g = LightGraphs.SimpleDiGraph(adj_mat)

    cc = LightGraphs.strongly_connected_components(g)

    # cc is an array of arrays
    depot_components = Vector{Vector{Int64}}(undef, 0)
    for c in cc
        if length(c) > 1
            this_comp_depots = Vector{Int64}(undef, 0)
            for idx in c
                if idx <= n_depots
                    push!(this_comp_depots, idx)
                end
            end

            # Each component MUST have a depot
            # @assert ~(isempty(this_comp_depots))

            push!(depot_components, this_comp_depots)
        end
    end

    return depot_components
end


function add_merged_depot_edges!(x_edges::Vector{Int64}, depot_comps::Vector{Vector{Int64}},
                                 depot_site_vector_idx::Dict{Tuple{Int64,Int64},Int64}, n_depots::Int64,
                                 n_sites::Int64, cost_vector::Vector{Float64})

    merged_depot_comps = depot_comps

    # Continue until one component
    while length(merged_depot_comps) > 1

        min_to_merge = (0, 0)
        min_depots_in_merged_comps = (0, 0)
        cmin = Inf

        for comp1_idx = 1:length(merged_depot_comps)-1
            for comp2_idx = comp1_idx+1 : length(merged_depot_comps)

                cmin_for_dep_pair = Inf
                depots_to_merge = (0, 0)

                for dep1 in merged_depot_comps[comp1_idx]
                    for dep2 in merged_depot_comps[comp2_idx]

                        dep_cost = cost_vector[depot_site_vector_idx[(dep1, dep2)]] +
                                    cost_vector[depot_site_vector_idx[(dep2, dep1)]]
                        if dep_cost < cmin_for_dep_pair
                            cmin_for_dep_pair = dep_cost
                            depots_to_merge = (dep1, dep2)
                        end
                    end
                end

                if cmin_for_dep_pair < cmin
                    cmin = cmin_for_dep_pair
                    min_to_merge = (comp1_idx, comp2_idx)
                    min_depots_in_merged_comps = depots_to_merge
                end
            end
        end

        # Add edges for merged deps
        dep1to2 = depot_site_vector_idx[(min_depots_in_merged_comps[1], min_depots_in_merged_comps[2])]
        @assert x_edges[dep1to2] == 0.0 "$(min_to_merge) \n $(merged_depot_comps)"
        x_edges[dep1to2] = 1.0
        dep2to1 = depot_site_vector_idx[(min_depots_in_merged_comps[2], min_depots_in_merged_comps[1])]
        @assert x_edges[dep2to1] == 0.0
        x_edges[dep2to1] = 1.0


        # Now create new merged depot comps
        new_comps = length(merged_depot_comps) - 1
        new_merged_depot_comps = Vector{Vector{Int64}}(undef, 0)

        push!(new_merged_depot_comps, vcat(merged_depot_comps[min_to_merge[1]], merged_depot_comps[min_to_merge[2]]))

        for (i, deps) in enumerate(merged_depot_comps)

            if ~(i in min_to_merge)
                push!(new_merged_depot_comps, deps)
            end
        end

        @assert length(new_merged_depot_comps) == new_comps

        merged_depot_comps = deepcopy(new_merged_depot_comps)
    end

    return merged_depot_comps
end


function get_multiedge_eulerian_tour(x_edges::Vector{Int64}, vector_idx_to_depot_site::Dict{Int64,Tuple{Int64,Int64}},
                                     n_depots::Int64, n_sites::Int64)

    edge_count = zeros(Int64, n_depots+n_sites)
    adj_list = [Int64[] for i = 1:n_depots+n_sites]

    for (i, val) in enumerate(x_edges)

        # Val is the number of edges
        if val > 0

            idx_pred, idx_succ = vector_idx_to_depot_site[i]

            # Add to the number of out_edges
            edge_count[idx_pred] += val

            # Append the neighbor val number of times
            for _ = 1:val
                push!(adj_list[idx_pred], idx_succ)
            end
        end
    end

    # Now we have the correct edge_count and adj_list
    curr_path = Stack{Int64}()
    circuit = Vector{Int64}(undef, 0)

    # Start from the first vtx with degree > 1 (should just be 1)
    first_idx = 0
    for i = 1:n_depots
        if edge_count[i] > 0
            first_idx = i
            break
        end
    end
    # Push now because will also end at it
    push!(curr_path, first_idx)
    curr_v = first_idx

    while ~(isempty(curr_path))

        if edge_count[curr_v] > 0

            push!(curr_path, curr_v)
            next_v = pop!(adj_list[curr_v])
            edge_count[curr_v] -= 1
            curr_v = next_v

        else

            push!(circuit, curr_v)
            curr_v = pop!(curr_path)

        end
    end

    return circuit
end

# Remove leading and trailing depots except the one just before the
# first package and the one just after the last package
# TODO: What if only depots???
function trim_circuit!(circuit::Vector, n_depots::Int64)

    # Trim leading depots
    first_site_idx = findfirst(x -> x > n_depots, circuit)

    if first_site_idx == nothing
        empty!(circuit)
        return
    end

    for i = 1:first_site_idx-2
        popfirst!(circuit)
    end

    last_site_idx = findlast(x -> x > n_depots, circuit)
    for i = last_site_idx+2:length(circuit)
        pop!(circuit)
    end

end

function cut_tours(circuit::Vector{Int64}, n_depots::Int64, n_agents::Int64,
                   depot_sites::Vector{LOC}, cost_fn::F) where {LOC, F <: Function}

    arc_costs = [cost_fn(depot_sites[i], depot_sites[j]) for (i, j) in zip(circuit[1:end-1], circuit[2:end])]
    total_cost = sum(arc_costs)

    agent_tours = [Int64[] for _ = 1:n_agents]
    idx = 1 # The idx is over circuit

    for i = 1:n_agents

        this_agent_tour = [circuit[idx]]
        tour_cost = 0

        while tour_cost <= total_cost/n_agents && idx < length(circuit)

            tour_cost += arc_costs[idx]
            idx += 1
            push!(this_agent_tour, circuit[idx])

        end

        # Continue the tour as required
        if idx < length(circuit)

            # @assert idx <= length(circuit) - 2 "Last agent has no package!"

            if circuit[idx] > n_depots # Ends at a site - add next depot
                idx += 1
                push!(this_agent_tour, circuit[idx])
            else
                # Now it ended at a depot. Only increase idx if next is ALSO a depot
                if circuit[idx + 1] <= n_depots
                    idx += 1
                end
            end
        end

        agent_tours[i] = this_agent_tour

        if idx == length(circuit)
            break
        end
    end

    return agent_tours
end

function task_allocation(n_depots::Int64, n_sites::Int64, n_agents, depot_sites::Vector{LOC}, cost_fn::F) where {LOC, F <: Function}

    edges, x_edges, vds, dsv, cv = min_connecting_tour(n_depots, n_sites, depot_sites, cost_fn)

    depot_comps = get_connected_depots(n_depots, n_sites, edges)

    add_merged_depot_edges!(x_edges, depot_comps, dsv, n_depots, n_sites, cv)

    circuit = get_multiedge_eulerian_tour(x_edges, vds, n_depots, n_sites)

    trim_circuit!(circuit, n_depots)

    agent_tours = cut_tours(circuit, n_depots, n_agents, depot_sites, cost_fn)

    # Trip the agent sub-tours also
    for (i, atour) in enumerate(agent_tours)
        if ~(isempty(atour))
            trim_circuit!(agent_tours[i], n_depots)
        end
    end

    return agent_tours

end



# This is only used by mapf layer
# Get the set of AgentTask instances as a tuple of origin, site, dest
function get_agent_task_set(agent_tours::Vector{Vector{Int64}}, n_depots::Int64,
                            n_sites::Int64)

    agent_tasks = AgentTask[]

    for (i, atour) in enumerate(agent_tours)

        if length(atour) < 3 # For anything other than dpd'
            continue
        end

        @assert atour[1] <= n_depots && atour[2] > n_depots && atour[3] <= n_depots

        push!(agent_tasks, (origin=atour[1], site=atour[2], dest=atour[3]))
    end

    return agent_tasks
end
