## Implement (E)CBS functions
function MultiAgentPathFinding.add_constraint!(cbase::MAPFTransitConstraints,
                                               cadd::MAPFTransitConstraints)
    union!(cbase.avoid_vertex_strs, cadd.avoid_vertex_strs)
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::MAPFTransitConstraints,
                                                           cadd::MAPFTransitConstraints)
    return ~(isempty(intersect(cbase.avoid_vertex_strs, cadd.avoid_vertex_strs)))
end

function MultiAgentPathFinding.get_mapf_state_from_idx(env::MAPFTransitEnv, idx::Int64)
    return env.state_graph.vertices[idx]
end

function get_mapf_transit_action(env::MAPFTransitEnv, vtx_u::MAPFTransitVertexState, vtx_v::MAPFTransitVertexState)

    usplit = split(vtx_u.vertex_str, "-")
    vsplit = split(vtx_v.vertex_str, "-")

    if usplit[1] == "r" && vsplit[1] == "r" &&
        usplit[2] == vsplit[2]
        return MAPFTransitAction(Stay::ActionType)
    elseif vsplit[1] != "r"
        return MAPFTransitAction(Fly::ActionType)
    else
        return MAPFTransitAction(Board::ActionType)
    end

end

function MultiAgentPathFinding.get_mapf_action(env::MAPFTransitEnv, u::Int64, v::Int64)

    vtx_u = env.state_graph.vertices[u]
    vtx_v = env.state_graph.vertices[v]

    return get_mapf_transit_action(env, vtx_u, vtx_v)

end


function MultiAgentPathFinding.set_low_level_context!(::MAPFTransitEnv, ::Int64, ::MAPFTransitConstraints)
    # Not sure what to do here
end

MultiAgentPathFinding.get_empty_constraint(::Type{MAPFTransitConstraints}) = Set{String}()


function MultiAgentPathFinding.get_first_conflict(env::MAPFTransitEnv,
                                                  solution::Vector{PlanResult})

    # First look for boarding/alighting constraints
    # But simultaneously note transit capacity usage
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])


            # Now we have the two solutions
            for (state_i, act_i) in zip(sol_i.states[2:end], sol_i.actions)
                for (state_j, act_j) in zip(sol_j.states[2:end], sol_j.actions)

                    # Conflict - boarding vertex at same time
                    # TODO : Prevent leaving at same time? One leaving, one boarding?
                    if state_i.state == state_j.state && act_i == Board::ActionType && act_j == Board::ActionType
                        return MAPFTransitConflict(type = Transfer::ConflictType,
                                                   overlap_vertices = Set{String}(state_i.vertex_str),
                                                   overlap_agents = Set{Int64}([i, j]))
                    end
                end
            end
        end
    end


    # Now look for capacity constraints ONLY
    transit_vtx_agents = Dict()
    for (i, sol_i) in enumerate(solution)
        for state_i in sol_i.states

            isplit = split(state_i.vertex_str, "-")

            if isplit[1] == "r"
                rid = parse(Int64, isplit[2])
                wpid = parse(Int64, isplit[3])

                if ~(haskey(transit_vtx_agents, rid))
                    transit_vtx_agents[rid] = Dict{Int64,Set{Int64}}()
                end

                if ~(haskey(transit_vtx_agents[rid], wpid))
                    transit_vtx_agents[rid][wpid] = Set{Int64}()
                end

                push!(transit_vtx_agents[rid][wpid], i)
            end
        end
    end

    # Now loop over vtx_agents and report first over-capacity link
    # TODO: Find a way to use all conflicts??? For now, returning first
    for (rid, wpid_agents) in transit_vtx_agents

        route_cap = transit_capacity[rid]
        wpid_list = sort(collect(keys(wpid_agents)))

        for i = 1:length(wpid_list)-1

            wpid = wpid_list[i]
            agents = wpid_agents[wpid]

            # Check if this marks the start of a link with excess capacity
            # If it does, get the particulars and return conflict
            if length(agents) > route_cap

                overlap_start = string("r-", rid, "-", wpid)
                overlap_verts = Set{String}(overlap_start)

                for j = i+1:length(wpid_list)

                    next_wpid = wpid_list[j]
                    next_agents = wpid_agents[next_wpid]
                    next_overlap = string("r-", rid, "-", next_wpid)
                    push!(overlap_verts, next_overlap)

                    if next_agents != agents
                        break
                    end
                end

                # Return conflict
                return MAPFTransitConflict(type = Capacity::ConflictType,
                                           overlap_vertices = overlap_verts,
                                           overlap_agents = agents,
                                           cap = route_cap)
            end
        end
    end

    # No conflict thus far
    return nothing

end

function MultiAgentPathFinding.create_constraints_from_conflict(env::MAPFTransitEnv, conflict::MAPFTransitConflict)

    # Do for each type of conflict
    if conflict.type ==  Transfer::ConflictType

        res_constraints = Dict{Int64,MAPFTransitConstraints}()
        for agt in conflict.overlap_agents
            res_constraints[agt] = MAPFTransitConstraints(conflict.overlap_vertices)
        end
        return [res_constraints]

    else

        route_cap = conflict.cap
        n_excess = length(conflict.overlap_agents) - route_cap
        res_constraint_set = Vector{Dict}(undef, 0)

        # Generate all subsets of excess agents and generate Constraints
        for agt_subset in subsets(collect(conflict.overlap_agents), n_excess)

            for agt in agt_subset
                res_constraints[agt] = MAPFTransitConstraints(conflict.overlap_vertices)
            end

            push!(res_constraint_set, res_constraints)
        end

        return res_constraint_set
    end
end


function elapsed_time(env::MAPFTransitEnv,
                      s1::MAPFTransitVertexState, s2::MAPFTransitVertexState)

    if s2.time == 0.0

        # Compute distance between s1 and s2 and divide by avg speed
        dist = env.dist_fn(s1.state.location, s2.state.location)
        return dist/env.drone_params.avg_speed

    else
        return (s2.time - s1.time)
    end
end

function elapsed_time_heuristic(env::MAPFTransitEnv, s::MAPFTransitVertexState)

    # Just direct flight time to goal
    goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
    return (env.dist_fn(goal_vtx.state.location, s.state.location)/env.drone_params.avg_speed)

end


function distance_traversed(env::MAPFTransitEnv, s1::MAPFTransitVertexState, s2::MAPFTransitVertexState)

    split1 = split(s1.vertex_str, "-")
    split2 = split(s2.vertex_str, "-")

    if split1[1] == "r" && split2[1] == "r" && split1[2] == split2[2]
        return 0.0
    else
        return env.dist_fn(s1.state.location, s2.state.location)
    end
end

# Heuristic used for both travel time and flight distance
function distance_heuristic(env::MAPFTransitEnv, sldist::Float64,
                            nn_stop::Int64, nn_dist::Float64, s::MAPFTransitVertexState)

    # Now do different things based on whether s is depot/site or vertex
    ssplit = split(s.vertex_str, "-")
    if ssplit[1] == "r"

        trip_id = parse(Int64, ssplit[2])

        # Iterate over possible trips for nn_stop and find nearest
        min_trip_to_trip = Inf
        for nn_trip_id in env.stop_idx_to_trips[nn_stop]
            min_trip_to_trip = (env.trips_fws_dists[trip_id, nn_trip_id] < min_trip_to_trip) ?  env.trips_fws_dists[trip_id, nn_trip_id] : min_trip_to_trip
        end

        # TODO : Remove
        @assert min_trip_to_trip != Inf
        return (nn_dist + min_trip_to_trip)
    else
        return sldist
    end
end


# Focal state heuristic - min boarding conflicts
function focal_transition_heuristic_transit(env::MAPFTransitEnv, solution::Vector{PlanResult},
                                            agent_idx::Int64, u::MAPFTransitVertexState, v::MAPFTransitVertexState)

    num_conflicts = 0

    this_act = get_mapf_transit_action(env, u, v)

    for (i, agt_soln) in enumerate(solution)
        if i != agent_idx && ~(isempty(agt_soln))

            # Count conflicts
            for (vtx_state, act) in zip(agt_soln.states[2:end], agt_soln.actions)
                if vtx_state.state == s2 && act == Board::ActionType && this_act == Board::ActionType
                    num_conflicts += 1
                end
            end
        end
    end

    return num_conflicts
end


function reachable_by_agent(env::MAPFTransitEnv, s1::MAPFTransitState, s2::MAPFTransitState)
    # Check that dist is less than avg_speed x time
    return env.drone_params.avg_speed * (s2.time - s1.time) > env.dist_fn(s1.location, s2.location)
end


# Runs low level search from depot to site, and then to depot
# Info for each agent available as agent_tasks
function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, agent_idx::Int64,
                                                 s::MAPFTransitState, constraints::MAPFTransitConstraints,
                                                 solution::Vector{PlanResult})

    env = solver.env

    # First run the search from origin to site
    # Reset the times of origin and site
    agt_task = env.agent_tasks[agent_idx]
    orig_str = string("d-", agt_task.origin)
    orig_idx = env.depot_sites_to_vtx[orig_str]
    reset_time(env.state_graph.vertices[orig_idx])

    goal_str = string("s-", agt_task.site)
    env.curr_goal_idx = env.depot_sites_to_vtx[goal_str]
    reset_time(env.state_graph.vertices[env.curr_goal_idx])

    # Need the following
    # Edge weight function - Just the elapsed time difference
    edge_wt_fn(u, v) = elapsed_time(env, env.state_graph.vertices[u], env.state_graph.vertices[v])

    # Edge constraint function
    edge_constr_fn(u, v) = distance_traversed(env, env.state_graph.vertices[u], env.state_graph.vertices[v])
    edge_constr_functions = [edge_constr_fn]

    edge_constraints = [env.drone_params.max_distance]

    # Admissible heuristics
    admissible_heuristic(u) = elapsed_time_heuristic(env, env.state_graph.vertices[u])

    # Inadmissible heuristics
    focal_state_heuristic(u) = 0.0
    focal_transition_heuristic(u, v) = focal_transition_heuristic_transit(env, solution, agent_idx, u, v)


    # Constraint heuristic
    sldist_heur = env.depot_to_sites_dists[agt_task.origin, agt_task.site]
    goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
    goal_vtx_loc_vect = convert_to_vector(goal_vtx.state.location)
    nn_idx, nn_dist = knn(env.stops_nn_tree, goal_vtx_loc_vect, 1)
    nn_stop = env.nn_idx_to_stop[nn_idx]
    dist_heur(u) = distance_heuristic(env, sldist_heur, nn_stop, nn_dist, env.state_graph.vertices[u])
    edge_constr_heuristics = [dist_heur]

    # RUN SEARCH
    states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                           edge_wt_fn,
                                                                           orig_idx, vis, solver.weight,
                                                                           admissible_heuristic,
                                                                           focal_state_heuristic,
                                                                           focal_transition_heuristic,
                                                                           edge_constr_functions,
                                                                           edge_constr_heuristics,
                                                                           edge_constraints)

    if tgt_entry.v_idx == orig_idx
        @warn "Agent $(agent_idx) Did not find path!"
        return PlanResult{MAPFTransitVertexState, MAPFTransitAction, Float64}()
    end

    # Get sp idxs, costs and weights
    sp_idxs, cost, wts = shortest_path_cost_weights(states, env.state_graph, orig_idx, tgt_entry)

    # Generate plan result
    states = [(get_mapf_state_from_idx(env, idx), 0.0) for idx in sp_idxs]
    actions = [(get_mapf_action(env, u, v), 0.0) for (u, v) in zip(sp_idxs[1:end-1], sp[2:end])]
    fmin = states.best_fvalue

    return PlanResult(states, actions, cost, fmin)
end


mutable struct MAPFTransitGoalVisitor <: AbstractDijkstraVisitor
    env::MAPFTransitEnv
    constraints::MAPFTransitConstraints
end


function Graphs.include_vertex!(vis::MAPFTransitGoalVisitor, u::MAPFTransitVertexState, v::MAPFTransitVertexState,
                                d::Float64, nbrs::Vector{Int64})

    env = vis.env
    goal_vtx = env.state_graph.vertices[env.curr_goal_idx]

    if goal_vtx.vertex_str == v.vertex_str
        # TODO: Check if ok to include time here
        extra_time = elapsed_time(env, u, v)
        reset_time(env.state_graph.vertices[env.curr_goal_idx], d + extra_time)
        return false
    end

    # Now generate neighbours
    # Always consider goal if possible to reach
    if env.dist_fn(v.state.location, goal_vtx.state.location) < env.drone_params.max_distance
        push!(nbrs, env.curr_goal_idx)
    end

    # Now add other neighbours according to whether depot/site or route vertex
    vsplit = split(v, "-")

    if vsplit[1] == "r"

        # Get the action leading to it
        act = get_mapf_transit_action(env, u, v)
        trip_id = parse(Int64, vsplit[2])

        vtx_range = env.trip_to_vtx_range[trip_id]

        next_vtx = vtx_range[1] + parse(Int64, vsplit[3]) + 1

        if act == Board::ActionType
            # If just boarded, just add the next route vertex
             # say 21-30, seq 4; add vtx 25 (5th in seq)
            push!(nbrs, next_vtx)
            return true
        else
            # Has been on trip - can either continue to next (if any left) and/or must get off
            if next_vtx <= vtx_range[2]
                push!(nbrs, next_vtx)
            end

            # Cycle through other trips and add those that can be reached in time
            for (tid, trip) in enumerate(env.transit_graph.transit_trips)

                # Ignore the current trip id
                if tid != trip_id
                    trip_vtx_range = env.trip_to_vtx_range[tid]

                    for seq = trip_vtx_range[1]:trip_vtx_range[2]
                        if reachable_by_agent(env, v.state, env.state_graph.vertices[seq].state)
                            push!(nbrs, seq)
                        end
                    end
                end
            end
        end
    else
        # A depot/site vertex - just add possible trip verts
        for (tid, trip) in enumerate(env.transit_graph.transit_trips)

            # Ignore the current trip id
            trip_vtx_range = env.trip_to_vtx_range[tid]

            for seq = trip_vtx_range[1]:trip_vtx_range[2]
                if reachable_by_agent(env, v.state, env.state_graph.vertices[seq].state)
                    push!(nbrs, seq)
                end
            end
        end
    end

    # @show nbrs

    return true
end
