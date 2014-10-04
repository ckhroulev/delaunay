### Delaunay triangulation ###

This is a re-implementation of Chip Collier's Commol Lisp code (<https://github.com/photex/lofi-tri>) in Julia. See the paper by Paul Bourke, (<http://paulbourke.net/papers/triangulate/>), for the description of the algorithm.

The code seems pretty robust, but requires a triangulation to add points to just to start the process. Currently we create a "super-triangle" that should contain all the points and use that as the starting triangulation, then remove dummy points from the result. The method used to create the super-triangle feels a bit questionable.

![a random triangulation](https://raw.githubusercontent.com/ckhroulev/delaunay/master/mesh.png)
