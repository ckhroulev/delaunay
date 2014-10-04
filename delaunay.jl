module Delaunay

export random_point_array, Triangle, triangulate

using Base.Graphics             # for Point (AKA Vec2) and BoundingBox

# dot-product of 2 vectors
dot(p :: Point, q :: Point) = p.x * q.x + p.y * q.y

# generate an array of random points in the [0,1]x[0,1] square
random_point_array(n) = [Point(x,y) for (x,y) in zip(rand(n), rand(n))]

immutable Circle
    center :: Point
    radius_squared :: Float64
    Circle(c :: Point, r :: Number) = new(c, float64(r^2))
end

function Circle(v0 :: Point, v1 :: Point, v2 :: Point)
    # Compute the circumcircle of 3 points on the plane.
    A = v1 - v0
    B = v2 - v0
    C = v2 - v1
    e = dot(A, v1 + v0)
    f = dot(B, v2 + v0)
    g = 2.0 * (A.x * C.y - A.y * C.x)
    colinearp = abs(g) < eps()
    if colinearp == true
        xmin = min(v0.x, v1.x, v2.x)
        ymin = min(v0.y, v1.y, v2.y)
        xmax = max(v0.x, v1.x, v2.x)
        ymax = max(v0.y, v1.y, v2.y)

        center = 0.5 * Point(xmin + xmax, ymin + ymax)
        radius = norm(center - Point(xmin, ymin))
    else
        center = Point((B.y*e - A.y*f) / g, (A.x*f - B.x*e) / g)
        radius = norm(center - v0)
    end
    Circle(center, radius)
end

immutable Triangle
    points :: Array{Point, 1}
    verts :: Array{Integer, 1}    # indexes of nodes in "points"
    circumcircle :: Circle
    Triangle(points, a :: Integer, b :: Integer, c :: Integer) = new(points, [a, b, c],
                                                                     Circle(points[a], points[b], points[c]))
end

Base.show(io::IO, t::Triangle) = print(io, "<$(join(t.verts, ','))>")

function get_bounding_triangle_points(point_set :: Array{Point, 1}, fudge_factor = 1)
    bbox = BoundingBox(point_set...)
    dx = fudge_factor * width(bbox)
    dy = fudge_factor * height(bbox)
    return [Point(bbox.xmin - dx, bbox.ymin - 3dy), # FIXME
            Point(bbox.xmin - dx, bbox.ymax + dy),
            Point(bbox.xmax + 3dx, bbox.ymax + dy)] # FIXME
end

distance_squared(p :: Point, q :: Point) = (p.x - q.x)^2 + (p.y - q.y)^2

function is_in_circumcircle(p :: Point, tri :: Triangle)
    # check if a point is within the circumcircle of a tringle
    return distance_squared(p, tri.circumcircle.center) <= tri.circumcircle.radius_squared
end

immutable Edge
    a :: Integer
    b :: Integer
end

==(A :: Edge, B :: Edge) = (A.a == B.a && A.b == B.b) || (A.a == B.b && A.b == B.a)
isequal(A :: Edge, B :: Edge) = (A == B)

function is_unique(a :: Edge, edges :: Array{Edge, 1})
    # Return 'true' if a appears at most once in edges, false
    # otherwise.
    first = true
    for e in edges
        if e == a
            if first == true    # found it for the first time
                first = false
            else
                return false    # found it for the second time: not unique
            end
        end            
    end
    return true                 # not found or found once: unique

    # equivalent call:
    # count(x -> x == a, edges) <= 1
    # this cannot bail early, though
end

function have_shared_verts(a :: Triangle, b :: Triangle)
    # Return true if triangles a and b share some vertexes.
    length(intersect(a.verts, b.verts)) >= 1
end


# After each point is added there is a net gain of two triangles. Thus
# the total number of triangles is twice the number of sample points.
# (This includes the supertriangle, when the triangles sharing edges
# with the supertriangle are deleted at the end the exact number of
# triangles will be less than twice the number of vertices, the exact
# number depends on the sample point distribution)

function add_vertex(vi :: Integer, triangles)
    # Add one vertex to a triangulation.
    edges = Array(Edge, 0)
    unaffected_triangles = Array(Triangle, 0)
    points = triangles[1].points

    for T in triangles
        if is_in_circumcircle(points[vi], T)
            verts = T.verts   # shortcut
            push!(edges, Edge(verts[1], verts[2]))
            push!(edges, Edge(verts[2], verts[3]))
            push!(edges, Edge(verts[3], verts[1]))
        else
            push!(unaffected_triangles, T)
        end
    end

    for E in edges
        if is_unique(E, edges)
            push!(unaffected_triangles, Triangle(points, vi, E.a, E.b))
        end
    end

    return unaffected_triangles
end

function triangulate(points :: Array{Point, 1})
    # add points of the bounding triangle to the set
    new_points = append!(deepcopy(points),
                         get_bounding_triangle_points(points))

    # add the bounding triangle
    N = length(points)
    triangles = Array(Triangle, 0)
    bounding_triangle = Triangle(new_points, N + 1, N + 2, N + 3)
    push!(triangles, bounding_triangle)

    for i = 1:N
        triangles = add_vertex(i, triangles)
    end

    filter(t -> ! have_shared_verts(t, bounding_triangle), triangles)
end

end                             # module Delaunay


