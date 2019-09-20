using MultiAgentAllocationTransit
using JLD2
using Plots
using Images


bg_map_fn = "./data/sf_bb_big.png"
sf_bb_fn = "./data/sfmta/sf_bb_viz_params.toml"
solution_fn = "./data/viz_soln_sf_5agts.jld2"
out_gif_name = "./data/viz_soln_sf_5agts.gif"
time_step = 7.0
fps = 24

depot_size = 15
site_size = 5
drone_size = 7

# Load background image
bg_img = Images.load(bg_map_fn)
ht, wd = size(bg_img)

sf_bb_params = parse_city_params(sf_bb_fn)

# set up background plot
bg_plot = Plots.plot(bg_img, xlims=(0,wd), ylims=(0,ht), legend=false, axis=nothing, border=:none, aspect_ratio = 1, background_color=:transparent)

# Load solution
@load solution_fn solution depots sites

plot_depots_sites!(bg_plot, wd, ht, sf_bb_params, depots, sites, depot_size, site_size)
#
max_time = maximum([s.cost for s in solution])
TIMESTEPS = convert(Int64, floor(max_time/time_step))
@show TIMESTEPS

anim = @animate for t = 1:TIMESTEPS

    @show t
    time_val = time_step * t
    p = render_drones(bg_plot, wd, ht, sf_bb_params, time_val, solution, drone_size)
    Plots.plot(p)

end

gif(anim, out_gif_name, fps = fps)
