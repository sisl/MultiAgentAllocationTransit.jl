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

MultiAgentPathFinding.get_mapf_action(env::MAPFTransitEnv, ::Int64, ::Int64) = MAPFTransitAction()

function MultiAgentPathFinding.set_low_level_context!(::MAPFTransitEnv, ::Int64, ::MAPFTransitConstraints)
    # Not sure what to do here
end

MultiAgentPathFinding.get_empty_constraint(MAPFTransitConstraints) = Set{String}()

function MultiAgentPathFinding.get_first_conflict(env::MAPFTransitEnv,
                                                  solution::Vector{PlanResult{MAPFTransitState,MAPFTransitAction,Float64}})

    # Compare pairwise solutions and get ?first? conflict
    # But get the fleshed out conflict

end
