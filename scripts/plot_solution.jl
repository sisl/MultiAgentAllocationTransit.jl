using MultiAgentAllocationTransit
using JLD2
using Plots
using Images


bg_map_fn = "./data/wdc_viz_bb.png"
bb_fn = "./data/wmata/wdc_params.toml"
solution_fn = "./data/viz_soln_wdc_110agts.jld2"
out_gif_name = "./data/viz_soln_wdc_110agts_600dpi.gif"
time_step = 10.0
fps = 24

depot_size = 15
site_size = 2
drone_size = 5

# Load background image
bg_img = Images.load(bg_map_fn)
ht, wd = size(bg_img)

sf_bb_params = parse_city_params(bb_fn)

# set up background plot
bg_plot = Plots.plot(bg_img, xlims=(0,wd), ylims=(0,ht), legend=false, axis=nothing, border=:none,
                     aspect_ratio = 1,
                     background_color=:transparent, dpi=600)

# Load solution
@load solution_fn solution depots initial_sites

plot_depots_sites!(bg_plot, wd, ht, sf_bb_params, depots, initial_sites, depot_size, site_size)

max_time = maximum([s.cost for s in solution])
TIMESTEPS = convert(Int64, floor(max_time/time_step))
@show TIMESTEPS

anim = @animate for t = 1:TIMESTEPS

    @show t
    time_val = time_step * t
    p = render_drones(bg_plot, wd, ht, sf_bb_params, time_val, solution, drone_size)
    Plots.plot(p, dpi=600)

end

gif(anim, out_gif_name, fps = fps)
