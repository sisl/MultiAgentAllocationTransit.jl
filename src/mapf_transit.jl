## Implement (E)CBS functions
function MultiAgentPathFinding.add_constraint!(cbase::MAPFTransitConstraints,
                                               cadd::MAPFTransitConstraints)
    union!(cbase.avoid_vertex_map[towards], cadd.avoid_vertex_map[towards])
    union!(cbase.avoid_vertex_map[from], cadd.avoid_vertex_map[from])
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::MAPFTransitConstraints,
                                                           cadd::MAPFTransitConstraints)
    return ~(isempty(intersect(cbase.avoid_vertex_idxs, cadd.avoid_vertex_idxs)))
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

MultiAgentPathFinding.get_empty_constraint(::Type{MAPFTransitConstraints}) = MAPFTransitConstraints()


function MultiAgentPathFinding.get_first_conflict(env::MAPFTransitEnv,
                                                  solution::Vector{PR}) where {PR <: PlanResult}

    # First look for boarding/alighting constraints
    # But simultaneously note transit capacity usage
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])


            # Now we have the two solutions
            for (si, ((state_i, _), (act_i, _))) in enumerate(zip(sol_i.states[2:end], sol_i.actions))
                for (sj, ((state_j, _), (act_j, _))) in enumerate(zip(sol_j.states[2:end], sol_j.actions))

                    # Conflict - boarding vertex at same time
                    if state_i.state == state_j.state && act_i == Board::ActionType && act_j == Board::ActionType
                        return MAPFTransitConflict(type = Transfer::ConflictType,
                                                   overlap_vertices = Set{String}(state_i.idx),
                                                   agent_to_state_idx = Dict(i => si+1, j => sj+1)) # +1 because enumerating from 2
                    end
                end
            end
        end
    end


    # Now look for capacity constraints ONLY
    # Dict of
    transit_vtx_agents = Dict()
    for (i, sol_i) in enumerate(solution)
        for (si, (state_i, _)) in enumerate(sol_i.states)

            isplit = split(state_i.vertex_str, "-")

            if isplit[1] == "r"
                rid = parse(Int64, isplit[2])
                wpid = parse(Int64, isplit[3])

                if ~(haskey(transit_vtx_agents, rid))
                    transit_vtx_agents[rid] = Dict{Int64,Dict{Int64,Int64}}()
                end

                if ~(haskey(transit_vtx_agents[rid], wpid))
                    transit_vtx_agents[rid][wpid] = Dict{Int64,Int64}()
                end

                # Create a mini dict for waypoint
                transit_vtx_agents[rid][wpid][i] = si
            end
        end
    end

    # Now loop over vtx_agents and report first over-capacity link
    # TODO: Find a way to use all conflicts??? For now, returning first
    for (rid, wpid_agent_dicts) in transit_vtx_agents

        route_cap = env.transit_graph.transit_capacity[rid]
        wpid_list = sort(collect(keys(wpid_agent_dicts)))
        trip_to_vtx_range = env.trip_to_vtx_range[rid]

        for i = 1:length(wpid_list)-1

            wpid = wpid_list[i]
            agent_dict = wpid_agent_dicts[wpid]

            # Check if this marks the start of a link with excess capacity
            # If it does, get the particulars and return conflict
            if length(agent_dict) > route_cap

                overlap_idx = trip_to_vtx_range[1] + wpid - 1
                overlap_verts = Set{Int64}(overlap_idx)
                agents = Set(keys(agent_dict))

                for j = i+1:length(wpid_list)

                    next_wpid = wpid_list[j]
                    next_agent_dict = wpid_agent_dicts[next_wpid]
                    next_overlap = trip_to_vtx_range[1] + next_wpid - 1
                    push!(overlap_verts, next_overlap)

                    if Set(keys(next_agent_dict)) != agents
                        break
                    end
                end

                # Return conflict
                # @show overlap_verts
                # @show agent_dict
                # readline()
                return MAPFTransitConflict(type = Capacity::ConflictType,
                                           overlap_vertices = overlap_verts,
                                           agent_to_state_idx = agent_dict,
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

        # Obtain the kind of subpath
        for (agt, state_idx) in conflcit.agent_to_state_idx
            if state_idx < env.curr_site_points[agt] # Must be greater or less
                res_constraints[agt] = MAPFTransitConstraints(Dict(towards => conflict.overlap_vertices,
                                                                   from => Set{Int64}()))
            else
                res_constraints[agt] = MAPFTransitConstraints(Dict(from => conflict.overlap_vertices,
                                                                   towards => Set{Int64}()))
            end
        end
        return [res_constraints]

    else

        route_cap = conflict.cap
        overlap_agents = Set(keys(conflict.agent_to_state_idx))
        n_excess = length(overlap_agents) - route_cap
        res_constraint_set = Vector{Dict}(undef, 0)

        # In one pass, get agent to constraint mapping
        agt_to_constraints_dict = Dict()

        for agt in overlap_agents
            state_idx = conflict.agent_to_state_idx[agt]

            if state_idx < env.curr_site_points[agt] # Must be greater or less
                agt_to_constraints_dict[agt] = MAPFTransitConstraints(Dict(towards => conflict.overlap_vertices,
                                                                   from => Set{Int64}()))
            else
                agt_to_constraints_dict[agt] = MAPFTransitConstraints(Dict(from => conflict.overlap_vertices,
                                                                   towards => Set{Int64}()))
            end
        end

        # Generate all subsets of excess agents and generate Constraints
        for agt_subset in subsets(collect(overlap_agents), n_excess)

            res_constraints = Dict{Int64,MAPFTransitConstraints}()

            # For each agent in subset, find if its halfway point crossed
            for agt in agt_subset
                res_constraints[agt] = agt_to_constraints_dict[agt]
            end

            push!(res_constraint_set, res_constraints)
        end

        # @show res_constraint_set[1]
        # @show res_constraint_set[2]
        # readline()

        return res_constraint_set
    end
end


function elapsed_time(env::MAPFTransitEnv,
                      s1::MAPFTransitVertexState, s2::MAPFTransitVertexState)

    if s2.state.time == 0.0

        # Compute distance between s1 and s2 and divide by avg speed
        dist = env.dist_fn(s1.state.location, s2.state.location)
        return dist/env.drone_params.avg_speed

    else
        return (s2.state.time - s1.state.time)
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

        return (nn_dist + min_trip_to_trip)
    else

        return sldist
    end
end


# Focal state heuristic - min boarding conflicts
function focal_transition_heuristic_transit(env::MAPFTransitEnv, solution::Vector{PR},
                                            agent_idx::Int64, u::MAPFTransitVertexState, v::MAPFTransitVertexState) where {PR <: PlanResult}

    num_conflicts = 0

    this_act = get_mapf_transit_action(env, u, v)

    for (i, agt_soln) in enumerate(solution)
        if i != agent_idx && ~(isempty(agt_soln))

            # Count conflicts
            for (vtx_state, act) in zip(agt_soln.states[2:end], agt_soln.actions)
                if vtx_state[1].state == v && act[1] == MAPFTransitAction(Board) && this_act == MAPFTransitAction(Board)
                    num_conflicts += 1
                end
            end
        end
    end

    return num_conflicts
end

# Just count the boarding conflicts, if any?
function MultiAgentPathFinding.focal_heuristic(env::MAPFTransitEnv, solution::Vector{PR}) where {PR <: PlanResult}

    num_conflicts = 0

    # First look for boarding/alighting constraints
    # But simultaneously note transit capacity usage
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])


            # Now we have the two solutions
            for (si, ((state_i, _), (act_i, _))) in enumerate(zip(sol_i.states[2:end], sol_i.actions))
                for (sj, ((state_j, _), (act_j, _))) in enumerate(zip(sol_j.states[2:end], sol_j.actions))

                    # Conflict - boarding vertex at same time
                    if state_i.state == state_j.state && act_i == Board::ActionType && act_j == Board::ActionType
                        num_conflicts += 1
                    end
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
# FURTHER tasks can be run as separate chunks
# Info for each agent available as agent_tasks
function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, agent_idx::Int64,
                                                 s::MAPFTransitVertexState, constraints::MAPFTransitConstraints,
                                                 solution::Vector{PR}) where {PR <: PlanResult}

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
    edge_wt_fn(u, v) = elapsed_time(env, u, v)

    # Edge constraint function
    edge_constr_fn(u, v) = distance_traversed(env, u, v)

    edge_constr_functions = [edge_constr_fn]

    edge_constraints = [env.drone_params.max_distance]
    # edge_constraints = [Inf]

    # Admissible heuristics
    admissible_heuristic(u) = elapsed_time_heuristic(env, u)

    # Inadmissible heuristics
    focal_state_heuristic(u) = 0.0
    focal_transition_heuristic(u, v) = focal_transition_heuristic_transit(env, solution, agent_idx, u, v)


    # Constraint heuristic
    # Only run first search if soln empty OR constraints in towards
    if isempty(solution) || ~(isempty(constraints.avoid_vertex_map[towards]))

        sldist_heur = env.depot_to_sites_dists[agt_task.origin, agt_task.site]
        goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
        goal_vtx_loc_vect = convert_to_vector(goal_vtx.state.location)
        nn_idxs, nn_dists = knn(env.stops_nn_tree, goal_vtx_loc_vect, 1)
        nn_idx = nn_idxs[1]
        nn_dist = nn_dists[1]
        nn_stop = env.nn_idx_to_stop[nn_idx]
        dist_heur(u) = distance_heuristic(env, sldist_heur, nn_stop, nn_dist, u)
        edge_constr_heuristics = [dist_heur]

        # RUN SEARCH

        vis = MAPFTransitGoalVisitor(env, constraints.avoid_vertex_map[towards])



        astar_eps_states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                               edge_wt_fn,
                                                                               orig_idx, vis, solver.weight,
                                                                               admissible_heuristic,
                                                                               focal_state_heuristic,
                                                                               focal_transition_heuristic,
                                                                               edge_constr_functions,
                                                                               edge_constr_heuristics,
                                                                               edge_constraints)

    ## A* epsilon
    # @time a_star_eps_states = a_star_light_epsilon_shortest_path_implicit!(env.state_graph,
    #                                                            edge_wt_fn, orig_idx,
    #                                                            vis, solver.weight,
    #                                                            admissible_heuristic,
    #                                                            focal_state_heuristic,
    #                                                            focal_transition_heuristic,
    #                                                            Float64)
    # sp_idxs = shortest_path_indices(a_star_eps_states.parent_indices, env.state_graph,
    #                                 orig_idx, env.curr_goal_idx)
    # @show sp_idxs
    # @show [env.state_graph.vertices[s] for s in sp_idxs]
    # readline()

    ## A* normal
    # @time a_star_states = a_star_light_shortest_path_implicit!(env.state_graph, edge_wt_fn, orig_idx, vis, admissible_heuristic)
    # sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.state_graph,
    #                                 orig_idx, env.curr_goal_idx)
    # @show sp_idxs
    # @show [env.state_graph.vertices[s] for s in sp_idxs]
    # @show a_star_states.dists[env.curr_goal_idx]
    # readline()

    ## A* - constrained
    # @time astar_states, tgt_entry = a_star_constrained_shortest_path_implicit!(env.state_graph,
    #                                                                        edge_wt_fn,
    #                                                                        orig_idx, vis,  admissible_heuristic,
    #                                                                        edge_constr_functions,
    #                                                                        edge_constr_heuristics,
    #                                                                        edge_constraints)
    # sp_idxs, cost, wts = shortest_path_cost_weights(astar_states, env.state_graph, orig_idx, tgt_entry)
    # @show sp_idxs
    # @show [env.state_graph.vertices[s] for s in sp_idxs]
    # @show cost
    # readline()

        if tgt_entry.v_idx == orig_idx
            @warn "Agent $(agent_idx) Did not find first sub-path!"

            # For debugging, just make a direct path
            cost = elapsed_time(env, s, goal_vtx)
            states = [(s, 0.0), (goal_vtx, cost)]
            actions = [(MAPFTransitAction(Fly), 0.0)]
            fmin = cost
            env.curr_site_points[agent_idx] = 2
        else

            # Get sp idxs, costs and weights
            sp_idxs, cost, wts = shortest_path_cost_weights(astar_eps_states, env.state_graph, orig_idx, tgt_entry)

            # Generate plan result for first leg
            states = [(get_mapf_state_from_idx(env, idx), cost) for idx in sp_idxs]
            actions = [(get_mapf_action(env, u, v), 0.0) for (u, v) in zip(sp_idxs[1:end-1], sp_idxs[2:end])]

            # TODO: Radioactive! Change back when possible
            # fmin = astar_eps_states.best_fvalue
            fmin = cost

            # Update the current site points for the agent
            env.curr_site_points[agent_idx] = length(sp_idxs)
        end
    else
        # Copy over previous values
        # Iterate until env.curr_site_points[agt]
        # @assert env.curr_site_points[agent_idx] > 0

        @debug "Copying over subpart towards of agent $(agent_idx)"

        states = solution[agent_idx].states[1 : env.curr_site_points[agent_idx]]
        actions = solution[agent_idx].actions[1 : env.curr_site_points[agent_idx]]
        cost = states[end][2] # Copy cost of last
        fmin = cost
    end

    # Now run second part of the search

    # Now set the new origin but DON'T reset
    orig_str = string("s-", agt_task.site)
    orig_idx = env.depot_sites_to_vtx[orig_str]

    # Now set goal and reset
    goal_str = string("d-", agt_task.dest)
    env.curr_goal_idx = env.depot_sites_to_vtx[goal_str]
    reset_time(env.state_graph.vertices[env.curr_goal_idx])


    if isempty(solution) || ~(isempty(constraints.avoid_vertex_map[from]))

        # Only need to reset the constraint heuristic
        sldist_heur = env.depot_to_sites_dists[agt_task.dest, agt_task.site]
        goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
        goal_vtx_loc_vect = convert_to_vector(goal_vtx.state.location)
        nn_idxs, nn_dists = knn(env.stops_nn_tree, goal_vtx_loc_vect, 1)
        nn_idx = nn_idxs[1]
        nn_dist = nn_dists[1]
        nn_stop = env.nn_idx_to_stop[nn_idx]
        dist_heur_2(u) = distance_heuristic(env, sldist_heur, nn_stop, nn_dist, u)
        edge_constr_heuristics_2 = [dist_heur_2]

        # RUn second search
        vis = MAPFTransitGoalVisitor(env, constraints.avoid_vertex_map[from])
        astar_eps_states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                               edge_wt_fn,
                                                                               orig_idx, vis, solver.weight,
                                                                               admissible_heuristic,
                                                                               focal_state_heuristic,
                                                                               focal_transition_heuristic,
                                                                               edge_constr_functions,
                                                                               edge_constr_heuristics_2,
                                                                               edge_constraints)

       if tgt_entry.v_idx == orig_idx
           @warn "Agent $(agent_idx) Did not find second sub-path!"
           cost_2 = elapsed_time(env, env.state_graph.vertices[orig_idx], goal_vtx)
           append!(states, [(env.state_graph.vertices[orig_idx], 0.0), (goal_vtx, cost)])
           append!(actions, [(MAPFTransitAction(Fly), 0.0)])
       else

            # Extract solution stuff
            sp_idxs, cost_2, wts = shortest_path_cost_weights(astar_eps_states, env.state_graph, orig_idx, tgt_entry)

            append!(states, [(get_mapf_state_from_idx(env, idx), cost_2) for idx in sp_idxs[2:end]]) # First element is same as last of prev
            append!(actions, [(get_mapf_action(env, u, v), 0.0) for (u, v) in zip(sp_idxs[1:end-1], sp_idxs[2:end])])
        end

        cost += cost_2
        fmin += cost_2

    else

        # @assert env.curr_site_points[agent_idx] > 0

        @debug "Copying over subpart from of agent $(agent_idx)"

        append!(states, solution[agent_idx].states[env.curr_site_points[agent_idx] + 1 : end])
        append!(actions, solution[agent_idx].actions[env.curr_site_points[agent_idx] : end])
        cost += states[end][2] # Copy cost of last
        fmin += states[end][2]
    end

    return PlanResult{MAPFTransitVertexState, MAPFTransitAction, Float64}(states, actions, cost, fmin)
end


mutable struct MAPFTransitGoalVisitor <: AbstractDijkstraVisitor
    env::MAPFTransitEnv
    avoid_vertex_idxs::Set{Int64}
end


function Graphs.include_vertex!(vis::MAPFTransitGoalVisitor, u::MAPFTransitVertexState, v::MAPFTransitVertexState,
                                d::Float64, nbrs::Vector{Int64})

    env = vis.env
    avoid_vertex_idxs = vis.avoid_vertex_idxs
    goal_vtx = env.state_graph.vertices[env.curr_goal_idx]

    if goal_vtx.vertex_str == v.vertex_str
        # TODO: Commented out to have a rolling horizon of trips
        # reset_time(env.state_graph.vertices[env.curr_goal_idx], d)
        return false
    end

    # Now add other neighbours according to whether depot/site or route vertex
    vsplit = split(v.vertex_str, "-")

    if vsplit[1] == "r"

        # Get the action leading to it
        act = get_mapf_transit_action(env, u, v)
        trip_id = parse(Int64, vsplit[2])

        vtx_range = env.trip_to_vtx_range[trip_id]

        next_vtx = vtx_range[1] + parse(Int64, vsplit[3])

        if act == MAPFTransitAction(Board) && next_vtx <= vtx_range[2] &&
            ~(next_vtx in avoid_vertex_idxs)
            # If just boarded, just add the next route vertex
             # say 21-30, seq 4; add vtx 25 (5th in seq)
            push!(nbrs, next_vtx)
            return true
        else
            # Has been on trip - can either continue to next (if any left) and/or must get off
            if next_vtx <= vtx_range[2] &&
                ~(next_vtx in avoid_vertex_idxs)
                push!(nbrs, next_vtx)
            end

            # Cycle through other trips and add those that can be reached in time
            for (tid, trip) in enumerate(env.transit_graph.transit_trips)

                # Ignore the current trip id
                if tid != trip_id

                    # Use non-dominated set
                    subseq = get_non_dominated_trip_points(env, avoid_vertex_idxs, v, tid)
                    append!(nbrs, subseq)
                end
            end
        end
    else
        # A depot/site vertex - just add possible trip verts
        for (tid, trip) in enumerate(env.transit_graph.transit_trips)

            subseq = get_non_dominated_trip_points(env, avoid_vertex_idxs, v, tid)
            append!(nbrs, subseq)
        end
    end

    # Now generate neighbours
    # Always consider goal if possible to reach
    if env.dist_fn(v.state.location, goal_vtx.state.location) < env.drone_params.max_distance
        # @info "Goal added by ", v.idx
        push!(nbrs, env.curr_goal_idx)
    end

    # @show v
    # @show nbrs

    return true
end
