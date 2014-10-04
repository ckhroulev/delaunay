module PlotMesh

export plot, set_context

using Base.Graphics
using Cairo
using Delaunay

function device_to_user_distance(context, d)
    center = user_to_device(context, 0, 0)
    result, dummy = device_to_user(context, center[1] + d, center[2] + 0)
    return result
end

function set_context(point_set, width, height)
    bbox = BoundingBox(point_set...)
    size = max(Base.Graphics.width(bbox),
               Base.Graphics.height(bbox))

    surface = CairoRGBSurface(width, height)
    context = CairoContext(surface)

    # set background before transformations:
    save(context)
    set_source_rgb(context, 1.0 * ones(3)...)   # white
    rectangle(context, 0.0, 0.0, width, height)
    fill(context)
    restore(context)

    translate(context, 0.5width, 0.5height)

    scaling = 0.5 * min(width, height) / size

    scale(context, scaling, scaling)

    return context
end

function plot(context, t :: Triangle)
    new_path(context)
    for p in t.points[t.verts]
        line_to(context, p.x, p.y)
    end
    close_path(context)
    stroke(context)

    # plot the circumcircle
    plot(context, t.circumcircle)
end

function plot(context, c :: Delaunay.Circle)
    save(context)

    set_line_width(context, 1.0)
    set_source_rgb(context, ones(3) * 0.2...)
    set_dash(context, [5.0])

    circle(context, c.center.x, c.center.y, sqrt(c.radius_squared))
    stroke_preserve(context)

    set_source_rgba(context, 0.0, 0.0, 0.0, 0.1)
    fill(context)

    restore(context)
end

function plot(context, p :: Point)
    r = device_to_user_distance(context, 3.0)
    circle(context, p.x, p.y, r)
    fill(context)
end

function plot_triangulation(pts, figure_size)
    # plot a Delaunay triangulation
    triangles = Delaunay.triangulate(pts)

    context = set_context(pts, figure_size, figure_size)

    set_source_rgb(context, zeros(3)...)
    set_line_width(context, 1.0)
    set_line_join(context, 1)

    for t in triangles
        plot(context, t)
    end

    set_source_rgb(context, 0.8, 0.0, 0.0)    # red

    for p in pts
        plot(context, p)
    end

    # bounding box
    bbox = BoundingBox(pts...)
    set_line_width(context, 2.0)
    set_source_rgba(context, 0.8, 0.0, 0.0, 0.5)    # red
    rectangle(context, bbox.xmin, bbox.ymin, width(bbox), height(bbox))
    stroke(context)

    return context
end

pts = Delaunay.random_point_array(20)
pts = [p - Point(0.5, 0.5) for p in pts]

context = plot_triangulation(pts, 800)

write_to_png(context.surface, "mesh.png")


end                             # module
