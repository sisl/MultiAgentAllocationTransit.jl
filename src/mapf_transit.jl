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

function MultiAgentPathFinding.get_mapf_action(env::MAPFTransitEnv, u::Int64, v::Int64)

    vtx_u = env.state_graph.vertices[u]
    vtx_v = env.state_graph.vertices[v]

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


function MultiAgentPathFinding.set_low_level_context!(::MAPFTransitEnv, ::Int64, ::MAPFTransitConstraints)
    # Not sure what to do here
end

MultiAgentPathFinding.get_empty_constraint(MAPFTransitConstraints) = Set{String}()


function MultiAgentPathFinding.get_first_conflict(env::MAPFTransitEnv,
                                                  solution::Vector{PlanResult{MAPFTransitState,MAPFTransitAction,Float64}})

    # First look for boarding/alighting constraints
    # But simultaneously note transit capacity usage
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])


            # Now we have the two solutions
            for (state_i, act_i) in zip(sol_i.states[2:end], sol_i.actions)
                for (state_j, act_j) in zip(sol_j.states[2:end], sol_j.actions)

                    # Conflict - boarding vertex at same time
                    # TODO : Prevent leaving at same time? One leaving, one boarding?
                    if state_i == state_j && act_i == Board::ActionType && act_j == Board::ActionType
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



# Runs low level search from depot to site, and then to depot
# Info for each agent available as agent_tasks
function MultiAgentPathFinding.low_level_search!(env::MAPFTransitEnv, agent_idx::Int64,
                                                 s::MAPFTransitState, constraints::MAPFTransitConstraints)

    #

end
