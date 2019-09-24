# MultiAgentAllocationTransit

Accompanying code repository for our ICRA 2020 submission 'Efficient Multi-Drone Delivery Using Transit Networks' ([ArXiv]()).
In this paper, we present a comprehensive algorithmic framework to operate a large fleet of drones to deliver
packages in an urban area while using transit networks to enhance their effective range. Please see the extended
version on ArXiv for the formulation, algorithms, experiments, and discussions.

**Note** - For those of you familiar with the Julia package manager, I provide a `Manifest.toml` because there are two custom dependencies: my fork of [Graphs.jl](https://github.com/Shushman/Graphs.jl) (which has various extensions to A*
with an implicit graph representation) and my [MultiAgentPathFinding.jl](https://github.com/Shushman/MultiAgentPathFinding.jl),
which implements Enhanced CBS. You can also just `add` those repos directly and then `dev` this one, instead of
instantiating the environment.
Also, there are several moving parts to the code, and the two main units, graph search and multi-agent path finding have been tested themselves.
Thus, I've been a bit lazy with testing here, but I might add some basic integration tests later.


## Setup
The `MultiAgentAllocationTransit` repository is set up as a package with its own environment in [Julia 1.0](https://julialang.org/downloads/). Look at **Using someone else's project** at the Julia [package manager documentation](https://julialang.github.io/Pkg.jl/v1/environments/#Using-someone-else's-project-1) for the basic idea. To get the code up and running (after having installed Julia), first `cd` into the `MultiAgentAllocationTransit` folder.
Then start the Julia REPL and go into [package manager](https://julialang.github.io/Pkg.jl/v1/getting-started/) mode by pressing `]`, followed by:
```shell
(v1.0) pkg> activate .
(MultiAgentAllocationTransit) pkg> instantiate
```
This will install the necessary dependencies and essentially reproduce the Julia environment required to make the package work. You can test this by exiting the package manager mode with the backspace key and then in the Julia REPL entering:
```shell
julia> using MultiAgentAllocationTransit
```
The full package should then pre-compile. AFTER this step, you can start [IJulia](https://github.com/JuliaLang/IJulia.jl) (install it if you have not already) and open up the root folder:
```shell
julia> using IJulia
julia> notebook(dir="./")
```
You can then run the `multi-drone-routing-example` notebook to get an idea of how to use the code for a specific domain. An overview of the code package itself is given below, after the illustrations.

### Illustrations

Below is a visualized scenario with 80 agents in San Francisco,
using the bus network and other parameters as described in the paper.
The black pentagons are depots, the grey rectangles are delivery
locations, blue circles are drones flying, and red circles are drones riding on transit. We
do not render the actual transit vehicles (buses) for clarity. Multiple drones may use a transit option
simultaneously; we can only render one red circle in that case.
_Locations are randomly generated within a bounding
box, and some of them may be slightly offshore_.

![SFMTA Example](img/viz_soln_sf_78agts_100dpi.gif)

Here is another example, with 110 agents, in the Washington DC Metropolitan Area:

![WMATA Example](img/viz_soln_wdc_110agts_100dpi.gif)


### Code Overview
TODO
