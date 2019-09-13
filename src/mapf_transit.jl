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

                        conflict = MAPFTransitConflict(type = Transfer::ConflictType,
                                                   overlap_vertices = Set{String}(state_i.idx),
                                                   agent_to_state_idx = Dict(i => si+1, j => sj+1)) # +1 because enumerating from 2
                        return conflict
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
                conflict = MAPFTransitConflict(type = Capacity::ConflictType,
                                           overlap_vertices = overlap_verts,
                                           agent_to_state_idx = agent_dict,
                                           cap = route_cap)
                # @show conflict.type
                return conflict
            end
        end
    end

    # No conflict thus far
    return nothing

end


function MultiAgentPathFinding.create_constraints_from_conflict(env::MAPFTransitEnv, conflict::MAPFTransitConflict)

    # Increment the number of global conflicts and throw error if above threshold
    env.num_global_conflicts += 1
    if env.num_global_conflicts > env.threshold_global_conflicts
        throw(DomainError("Too many conflicts!"))
    end

    # Do for each type of conflict
    if conflict.type ==  Transfer::ConflictType

        res_constraints = Dict{Int64,MAPFTransitConstraints}()

        # Obtain the kind of subpath
        for (agt, state_idx) in conflict.agent_to_state_idx
            if state_idx < env.curr_site_points[agt] # Must be greater or less
                res_constraints[agt] = MAPFTransitConstraints(Dict(towards => conflict.overlap_vertices,
                                                                   from => Set{Int64}()))
            else
                res_constraints[agt] = MAPFTransitConstraints(Dict(from => conflict.overlap_vertices,
                                                                   towards => Set{Int64}()))
            end
        end

        # @show [res_constraints]
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

        # @show res_constraint_set
        return res_constraint_set
    end
end


function elapsed_time(env::MAPFTransitEnv,
                      s1::MAPFTransitVertexState, s2::MAPFTransitVertexState)

    if startswith(s2.vertex_str, "r") == true
        return (s2.state.time - s1.state.time)
    else
        # Compute distance between s1 and s2 and divide by avg speed
        dist = env.dist_fn(s1.state.location, s2.state.location)
        return dist/env.drone_params.avg_speed
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
function distance_heuristic(env::MAPFTransitEnv, nn_dist::Float64, nn_stop::Int64, s::MAPFTransitVertexState)

    # Now do different things based on whether s is depot/site or vertex
    ssplit = split(s.vertex_str, "-")
    if ssplit[1] == "r"

        trip_id = parse(Int64, ssplit[2])

        # Iterate over possible trips for nn_stop and find nearest
        min_trip_to_trip = Inf
        for nn_trip_id in env.stop_idx_to_trips[nn_stop]
            min_trip_to_trip = (env.trips_fws_dists[trip_id, nn_trip_id] < min_trip_to_trip) ?  env.trips_fws_dists[trip_id, nn_trip_id] : min_trip_to_trip
        end

        return nn_dist + min_trip_to_trip
    else
        # TODO: Keeping 0 for simplicity, as depot/site always popped. Change later
        return 0.0
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

    num_potential_conflicts = 0

    # First look for boarding/alighting constraints
    # But simultaneously note transit capacity usage
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])


            # Now we have the two solutions
            for (si, ((state_i, _), (act_i, _))) in enumerate(zip(sol_i.states[2:end], sol_i.actions))
                for (sj, ((state_j, _), (act_j, _))) in enumerate(zip(sol_j.states[2:end], sol_j.actions))

                    # Conflict - boarding vertex at same time
                    if state_i.state == state_j.state
                        num_potential_conflicts += 1
                    end
                end
            end
        end
    end

    return num_potential_conflicts
end

function reachable_by_agent(env::MAPFTransitEnv, s1::MAPFTransitState, s2::MAPFTransitState)
    # Check that dist is less than avg_speed x time
    return env.drone_params.avg_speed * (s2.time - s1.time) > env.dist_fn(s1.location, s2.location)
end


# Run low level search from current state for agent_idx
# Agent task is obtained from env.agent_states
# If site is crossed, don't need to care about that
# Search is done from the current env reference time (which accounts for rolling horizon)
function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, agent_idx::Int64,
                                                 s::MAPFTransitVertexState, constraints::MAPFTransitConstraints,
                                                 solution::Vector{PR}) where {PR <: PlanResult}

    env = solver.env
    agent_state = env.agent_states[agent_idx]

    # set up heuristics regardless
    # Set up the weight and heuristic functions
    edge_wt_fn(u, v) = elapsed_time(env, u, v)

    edge_constr_fn(u, v) = distance_traversed(env, u, v)
    edge_constr_functions = [edge_constr_fn]

    edge_constraints = [env.drone_params.max_distance - agent_state.dist_flown] # TODO: Check this!!!

    admissible_heuristic(u) = elapsed_time_heuristic(env, u)

    focal_state_heuristic(u) = 0.0
    focal_transition_heuristic(u, v) = focal_transition_heuristic_transit(env, solution, agent_idx, u, v)


    # Don't reset initial time
    orig_idx = s.idx

    states = Tuple{MAPFTransitVertexState,Float64}[]
    actions = Tuple{MAPFTransitAction,Float64}[]

    first_cost = env.plan_ref_times[agent_idx]

    # ONLY run first search if site not crossed
    if agent_state.site_crossed == false

        # Then goal is the site
        goal_str = goal_str = string("s-", agent_state.task.site)
        env.curr_goal_idx = env.depot_sites_to_vtx[goal_str]


        # Note - don't really need an sl dist heuristic
        # Since you are beginning the search anyway
        if isempty(solution) || ~(isempty(constraints.avoid_vertex_map[towards]))

            goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
            goal_vtx_loc_vect = convert_to_vector(goal_vtx.state.location)
            nn_idxs, nn_dists = knn(env.stops_nn_tree, goal_vtx_loc_vect, 1)
            nn_idx = nn_idxs[1]
            nn_dist = nn_dists[1]
            nn_stop = env.nn_idx_to_stop[nn_idx]
            dist_heur_1(u) = distance_heuristic(env, nn_dist, nn_stop, u)

            vis = MAPFTransitGoalVisitor(env, constraints.avoid_vertex_map[towards])



            astar_eps_states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                                   edge_wt_fn,
                                                                                   orig_idx, vis, solver.weight,
                                                                                   admissible_heuristic,
                                                                                   focal_state_heuristic,
                                                                                   focal_transition_heuristic,
                                                                                   edge_constr_functions,
                                                                                   [dist_heur_1],
                                                                                   edge_constraints)

            # IMP - Need to sort out solution
            if tgt_entry.v_idx == orig_idx
                @warn "Agent $(agent_idx) Did not find first sub-path!"

                env.any_invalid_path = true

                # For debugging, make a direct path with distance
                weight = distance_traversed(env, s, goal_vtx)

                append!(states, [(s, first_cost), (goal_vtx, first_cost + elapsed_time(env, s, goal_vtx))])
                append!(actions, [(MAPFTransitAction(Fly), weight)])
                env.curr_site_points[agent_idx] = 2

            else
                sp_idxs, costs, wts = shortest_path_cost_weights(astar_eps_states, env.state_graph, orig_idx, tgt_entry)

                append!(states, [(get_mapf_state_from_idx(env, idx), first_cost + c) for (idx, c) in zip(sp_idxs, costs)])
                append!(actions, [(get_mapf_action(env, u, v), wts[i+1][1] - wts[i][1]) for (i, (u, v)) in enumerate(zip(sp_idxs[1:end-1], sp_idxs[2:end]))])

                env.curr_site_points[agent_idx] = length(sp_idxs)
            end

        else
            # Copy over previous values
            @debug "Copying over subpart towards of agent $(agent_idx)"

            append!(states, solution[agent_idx].states[1 : env.curr_site_points[agent_idx]])
            append!(actions, solution[agent_idx].actions[1 : env.curr_site_points[agent_idx] - 1])
        end

        first_cost = states[end][2]

        # Reset stateful things for next search
        orig_str = string("s-", agent_state.task.site)
        orig_idx = env.depot_sites_to_vtx[orig_str]
        edge_constraints = [env.drone_params.max_distance]

    end



    goal_str = string("d-", agent_state.task.dest)
    env.curr_goal_idx = env.depot_sites_to_vtx[goal_str]

    if isempty(solution) || ~(isempty(constraints.avoid_vertex_map[from]))

        goal_vtx = env.state_graph.vertices[env.curr_goal_idx]
        goal_vtx_loc_vect = convert_to_vector(goal_vtx.state.location)
        nn_idxs, nn_dists = knn(env.stops_nn_tree, goal_vtx_loc_vect, 1)
        nn_idx = nn_idxs[1]
        nn_dist = nn_dists[1]
        nn_stop = env.nn_idx_to_stop[nn_idx]
        dist_heur_2(u) = distance_heuristic(env, nn_dist, nn_stop, u)


        vis = MAPFTransitGoalVisitor(env, constraints.avoid_vertex_map[from])
        astar_eps_states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                               edge_wt_fn,
                                                                               orig_idx, vis, solver.weight,
                                                                               admissible_heuristic,
                                                                               focal_state_heuristic,
                                                                               focal_transition_heuristic,
                                                                               edge_constr_functions,
                                                                               [dist_heur_2],
                                                                               edge_constraints)


        if tgt_entry.v_idx == orig_idx
            @warn "Agent $(agent_idx) Did not find second sub-path!"

            env.any_invalid_path = true

            weight = distance_traversed(env, env.state_graph.vertices[orig_idx], goal_vtx)

            append!(states, [(goal_vtx, first_cost + elapsed_time(env, env.state_graph.vertices[orig_idx], goal_vtx))])
            append!(actions, [(MAPFTransitAction(Fly),weight)])

        else

            sp_idxs, costs, wts = shortest_path_cost_weights(astar_eps_states, env.state_graph, orig_idx, tgt_entry)

            if agent_state.site_crossed == false
                append!(states, [(get_mapf_state_from_idx(env, idx), first_cost + c) for (idx, c) in zip(sp_idxs[2:end], costs[2:end])])
            else
                # Append the full post soln
                append!(states, [(get_mapf_state_from_idx(env, idx), first_cost + c) for (idx, c) in zip(sp_idxs, costs)])
            end

            append!(actions, [(get_mapf_action(env, u, v), wts[i+1][1] - wts[i][1]) for (i, (u, v)) in enumerate(zip(sp_idxs[1:end-1], sp_idxs[2:end]))])
        end

    else

        @debug "Copying over subpart from of agent $(agent_idx)"

        append!(states, solution[agent_idx].states[env.curr_site_points[agent_idx] + 1 : end])
        append!(actions, solution[agent_idx].actions[env.curr_site_points[agent_idx] : end])
    end

    cost = states[end][2]
    fmin = cost

    # Update the next finish time of agent
    env.agent_states[agent_idx].next_finish_time = cost

    return PlanResult{MAPFTransitVertexState, MAPFTransitAction, Float64}(states, actions, cost, fmin)
end

# Returns the next state of each agent based on its currently executing solution
# Also updates the site_crossed flag if so
# TODO: Assumes that solution[i].states has the correct tuple of (state, cost)
# Also assumes that agent has not finished yet
# Also assumes that solution[i].actions tracks the weight costs
function update_agent_states!(env::MAPFTransitEnv, time_val::Float64, agent_idx::Int64,
                              solution::Vector{PR}) where {PR <: PlanResult}

    @assert time_val < env.agent_states[agent_idx].next_finish_time "Agent $(agent_idx) has already finished at time $(time)!"

    agt_soln_states = solution[agent_idx].states
    agt_soln_actions = solution[agent_idx].actions

    state_idx = 0
    state_time = Inf

    # Iterate over agent_states until first state after time
    for (si, (state, t)) in enumerate(agt_soln_states)
        if t >= time_val
            state_idx = si
            state_time = t
            break
        end
    end

    # Now deal with state_idx
    @assert state_idx <= length(agt_soln_states) "Agent $(agent_idx) has inconsistent time trajectory!; $(time_val); $(agt_soln_states)"

    # TODO: hack to avoid pathological case of agent being at end of solution
    if state_idx == length(agt_soln_states)
        state_idx = state_idx - 1
        state_time = agt_soln_states[state_idx][2]
    end


    dist_start_idx = 1
    if state_idx >= env.curr_site_points[agent_idx]
        env.agent_states[agent_idx].site_crossed = true
        dist_start_idx = env.curr_site_points[agent_idx]
    end

    dist_flown = 0.0
    for i = dist_start_idx:state_idx-1
        dist_flown += agt_soln_actions[i][2]
    end

    env.agent_states[agent_idx].dist_flown = dist_flown

    return agt_soln_states[state_idx][1], state_time # That's the state
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



# Run a single search over a snapshot of the transit network to get the travel time
# If not reachable, then return Inf
function get_depot_to_site_travel_time(env::MAPFTransitEnv, weight::Float64, orig_str::String, goal_str::String)

    # Get origin idx and destination idx
    # No need to reset as must be zero
    orig_idx = env.depot_sites_to_vtx[orig_str]
    env.curr_goal_idx = env.depot_sites_to_vtx[goal_str]

    edge_wt_fn(u, v) = elapsed_time(env, u, v)
    edge_constr_fn(u, v) = distance_traversed(env, u, v)
    edge_constr_functions = [edge_constr_fn]
    edge_constraints = [env.drone_params.max_distance]
    admissible_heuristic(u) = elapsed_time_heuristic(env, u)
    focal_state_heuristic(u) = 0.0
    focal_transition_heuristic(u, v) = 0.0
    dist_heur(u) = 0.0  # Keep this simple
    edge_constr_heuristics = [dist_heur]

    vis = MAPFTransitGoalVisitor(env, Set{Int64}())

    astar_eps_states, tgt_entry = a_star_epsilon_constrained_shortest_path_implicit!(env.state_graph,
                                                                           edge_wt_fn,
                                                                           orig_idx, vis, weight,
                                                                           admissible_heuristic,
                                                                           focal_state_heuristic,
                                                                           focal_transition_heuristic,
                                                                           edge_constr_functions,
                                                                           edge_constr_heuristics,
                                                                           edge_constraints)

    # TODO: Not returning Inf to avoid instability?
    if tgt_entry.v_idx == orig_idx
        @warn "In allocation, edge from $(orig_str) to $(goal_str) is empty!"
        return 100000.
    end


    sp_idxs, costs, wts = shortest_path_cost_weights(astar_eps_states, env.state_graph, orig_idx, tgt_entry)

    return costs[end]
end


function allocation_cost_wrapper_truett(env::MAPFTransitEnv, weight::Float64, n_depots::Int64, n_sites::Int64,
                                    orig_ds_idx::Int64, goal_ds_idx::Int64)

    # Compute the origin and goal strings
    if orig_ds_idx > n_depots
        orig_str = string("s-", (orig_ds_idx - n_depots))
    else
        orig_str = string("d-", orig_ds_idx)
    end

    if goal_ds_idx > n_depots
        goal_str = string("s-", (goal_ds_idx - n_depots))
    else
        goal_str = string("d-", goal_ds_idx)
    end

    return get_depot_to_site_travel_time(env, weight, orig_str, goal_str)
end

function allocation_cost_wrapper_estimate(env::MAPFTransitEnv, weight::Float64, n_depots::Int64, n_sites::Int64,
                                          halton_nn_tree::BallTree, estimate_matrix::Matrix{Float64},
                                          orig_ds_idx::Int64, goal_ds_idx::Int64)

    # Compute the origin and goal strings
    if orig_ds_idx > n_depots
        orig_str = string("s-", (orig_ds_idx - n_depots))
    else
        orig_str = string("d-", orig_ds_idx)
    end
    loc1 = env.state_graph.vertices[env.depot_sites_to_vtx[orig_str]].state.location

    if goal_ds_idx > n_depots
        goal_str = string("s-", (goal_ds_idx - n_depots))
    else
        goal_str = string("d-", goal_ds_idx)
    end
    loc2 = env.state_graph.vertices[env.depot_sites_to_vtx[goal_str]].state.location

    tt = get_travel_time_estimate(halton_nn_tree, loc1, loc2, estimate_matrix)

    # if tt == 0. || tt == 100000.
    #     tt = env.dist_fn(loc1, loc2) / env.drone_params.
    # end

    return get_travel_time_estimate(halton_nn_tree, loc1, loc2, estimate_matrix)
end


## Replanning Heuristics


# Get the agent that will finish first
# Also returns the time at which it finishes
function get_first_finish(solution::Vector{PR}) where {PR <: PlanResult}

    min_cost = Inf
    min_idx = 0

    for (i, pr) in enumerate(solution)
        if pr.cost < min_cost
            min_idx = i
            min_cost = pr.cost
        end
    end

    return (min_idx, min_cost)
end

# For single agent replanning, we need the set of all vertex constraints of other
# agents to ignore as constraints to the replanning agent
# TODO: How to handle the time issue here? (Just take route vertex indices)
function get_replan_constraints(env::MAPFTransitEnv, time_val::Float64, agent_idx::Int64,
                              solution::Vector{PR}) where {PR <: PlanResult}

    avoid_vertex_idxs = Set{Int64}()

    # Iterate over all solutions for agents other than agent_idx and find vertices to avoid
    for (other_idx, soln) in enumerate(solution)

        if other_idx != agent_idx
            agt_soln_states = soln.states

            for (si, (state, t)) in enumerate(agt_soln_states)
                if t >= time_val
                    push!(avoid_vertex_idxs, state.idx)
                end
            end
        end
    end

    avoid_vertex_map = Dict(towards=>avoid_vertex_idxs, from=>avoid_vertex_idxs)

    return MAPFTransitConstraints(avoid_vertex_map)
end

# Get all the info necessary to replan for INDIVIDUAL
# Returns false if drone has no further task
# Also returns the planning time
function replan_individual!(env::MAPFTransitEnv, solution::Vector{PR},
                            n_depots::Int64, n_sites::Int64,
                            agent_tours::Vector{Vector{Int64}}, weight::Float64) where {PR <: PlanResult}

    # Get first finishing agent and next task for it
    replan_idx, time_val = get_first_finish(solution)
    finished_site = env.agent_states[replan_idx].task.site
    replan_task = get_next_agent_task(agent_tours[replan_idx], n_depots, n_sites, finished_site)

    if replan_task == nothing
        @warn "Agent has no next task!"
        return (false, 0.0)
    end

    env.agent_states[replan_idx] = AgentState(task=replan_task)
    new_init_state = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", replan_task.origin)]]
    replan_constraints = get_replan_constraints(env, time_val, replan_idx, solution)

    # Just a sham solver to rerun low level search
    solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = weight)
    el_time = @elapsed new_pr = MultiAgentPathFinding.low_level_search!(solver, replan_idx, new_init_state, replan_constraints, solution)
    solution[replan_idx] = new_pr

    return (true, el_time)
end


# Replan collectively! In this we want to update the agent states for all agents
# and then re-run the solver
# Returns true if valid, the elapsed time, and the new solution
function replan_collective!(env::MAPFTransitEnv, solution::Vector{PR},
                            n_depots::Int64, n_sites::Int64,
                            agent_tours::Vector{Vector{Int64}}, weight::Float64) where {PR <: PlanResult}


    replan_idx, time_val = get_first_finish(solution)
    finished_site = env.agent_states[replan_idx].task.site
    replan_task = get_next_agent_task(agent_tours[replan_idx], n_depots, n_sites, finished_site)

    if replan_task == nothing
        @warn "Agent has no next task!"
        return (false, 0.0, solution)
    end

    env.agent_states[replan_idx] = AgentState(task=replan_task)

    true_n_agents = length(agent_tours)
    new_initial_states = Vector{MAPFTransitVertexState}(undef, true_n_agents)

    for i = 1:true_n_agents
        if i == replan_idx
            new_initial_states[i] = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", replan_task.origin)]]
            env.plan_ref_times[i] = 0.0
        else
            new_state, ref_time = update_agent_states!(env, time_val, i, solution)
            new_initial_states[i] = new_state
            env.plan_ref_times[i] = ref_time
        end
    end

    solver = ECBSSolver{MAPFTransitVertexState,MAPFTransitAction,Float64,Makespan,MAPFTransitConflict,MAPFTransitConstraints,MAPFTransitEnv}(env = env, weight = weight)

    el_time = @elapsed new_solution = search!(solver, new_initial_states)

    return (true, el_time, new_solution)
end

## Set the number of transit options and valid distances
function set_solution_diagnostics!(env::MAPFTransitEnv, solution::Vector{PR}) where {PR <: PlanResult}

    # Iterate over solutions and only update if valid
    for (agent_idx, agent_soln) in enumerate(solution)

        # You know start state and site state - get dist
        agt_task = env.agent_states[agent_idx].task
        start_vtx = env.state_graph.vertices[env.depot_sites_to_vtx[string("d-", agt_task.origin)]]
        site_vtx = env.state_graph.vertices[env.depot_sites_to_vtx[string("s-", agt_task.site)]]
        dist = distance_traversed(env, start_vtx, site_vtx)

        # Now only update IF valid
        if dist < env.drone_params.max_distance || env.curr_site_points[agent_idx] > 2
            # IS VALID
            push!(env.valid_path_dists, dist)

            # Now search for transit options
            transit_opts = 0
            for (act, _) in agent_soln.actions
                if act.action == Board::ActionType
                    transit_opts += 1
                end
            end
            push!(env.valid_transit_options, transit_opts)
        end
    end
end
