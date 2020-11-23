#
# Main function for Hybrid A* Trailer 
# 
# Author: Atsushi Sakai(@Atsushi_twi)
#

using PyPlot

include("../src/trailer_hybrid_a_star.jl")

function read_obstacles()
    # f = open( "dat.txt", "r" )
    # n = countlines( f )
    # seekstart( f )

    # for i = 1:n
    #     x, y = split( readline( f ), "," )
    #     x = parse( x )
    #     y = parse( y )
    #     println( "x = ", x, ";", " y = ", y )
    # end

    # close( f )
end
function main()
    println(PROGRAM_FILE," start!!")

    # initial state
    sx = -15.0  # [m]
    sy = 10.0  # [m]
    syaw0 = deg2rad(0.0)
    syaw1 = deg2rad(0.0)

    # goal state
    gx = 0.0  # [m]
    gy = -5.0  # [m]
    gyaw0 = deg2rad(-90.0)
    gyaw1 = deg2rad(-90.0)

    # set obstacles
    ox = Float64[]
    oy = Float64[]

    # 

    bottom_y = -30.0

    for i in -25:25
        push!(ox, Float64(i))
        push!(oy, 15.0)
    end

    for i in -25:-5
        push!(ox, Float64(i))
        push!(oy, 5)
    end

    for i in bottom_y:5
        push!(ox, -5)
        push!(oy, Float64(i)) 
    end

    for i in bottom_y:15
        push!(ox, 5.0)
        push!(oy, Float64(i))
    end

    for i in 5:15
        push!(ox, -25)
        push!(oy, Float64(i))
    end

    oox = ox[:]
    ooy = oy[:]

    # trailer_hybrid_a_star.trailerlib.plot_trailer.(gx, gy, gyaw0, gyaw1, 0)
    # plot(ox, oy, ".k")

    
    # axis("equal")
    # show()

    # path generation
    path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,
                                                               trailer_hybrid_a_star.XY_GRID_RESOLUTION,
                                                               trailer_hybrid_a_star.YAW_GRID_RESOLUTION
                                                               )

    # ====Animation=====
    show_animation(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)

    println(PROGRAM_FILE," Done!!")
end


function show_animation(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
    plot(oox, ooy, ".k")
    trailer_hybrid_a_star.trailerlib.plot_trailer(sx, sy, syaw0, syaw1, 0.0)
    trailer_hybrid_a_star.trailerlib.plot_trailer(gx, gy, gyaw0, gyaw1, 0.0)
    x = path.x
    y = path.y
    yaw = path.yaw
    yaw1 = path.yaw1
    direction = path.direction

    steer = 0.0
    for ii in 1:length(x)
        cla()
        plot(oox, ooy, ".k")
        plot(x, y, "-r", label="Hybrid A* path")

        if ii < length(x)-1
            k = (yaw[ii+1] - yaw[ii])/trailer_hybrid_a_star.MOTION_RESOLUTION
            if !direction[ii]
                k *= -1
            end
            steer = atan(trailer_hybrid_a_star.WB*k, 1.0)
        else
            steer = 0.0
        end

        trailer_hybrid_a_star.trailerlib.plot_trailer.(x[ii], y[ii], yaw[ii], yaw1[ii], steer)
        grid(true)
        axis("equal")
        # show()
        # break
        pause(0.0005)


    end
end


# main()

if length(PROGRAM_FILE)!=0 &&
	occursin(PROGRAM_FILE, @__FILE__)
    main()
end

