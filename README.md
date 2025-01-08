## Introduction

This is an algorithm to perform a height query for a terrain mesh written in `rust`. The algorithm parses an .obj mesh, constructs a binary search tree acceleration structure and has a mesh traversal function returning the z height for a given x/y coordinate. It is designed to work for lookups in the x/y plane, i.e. it works in 2D assuming returning the corresponding height. It has support for multi-layered meshes and will return the greatest z value of stacked meshes. It could be extended to support bridges, this will require an initial guess for the z value and rejecting z-heights above a certain threshold with respect to the initial guess (not yet implemented). The algorithm will return `-inf` if there was no underlying triangle.
## Background

The acceleration structure is a loose k-D tree, which sorts along a single axis determined based on the largest bounds (either along the x or y axis). The group of triangles is split in two groups and the min/max bounds of the groups are stored in the node data. A tree is built until we have only nodes left in the leaves.

The reason why this structure was chosen over a boundary volume hierarchy tree or a quadtree, is because it was specifically designed for large triangle terrain meshes (such as road surfaces or circuits). It is designed to have a balanced tree with a consistent runtime performance.

## Build and Run

Build and run using cargo:

```shell
$ cargo run --release
```

Test coverage:
```shell
$ cargo llvm-cov --html && open target/llvm-cov/html/index.html
```

**Note**: Unzip the included `.obj` file in the `./data` folder beforehand.

Make sure to run it in `release` mode, as it will run significantly slower in `debug` mode.

**NOTE:** A `c++` version of the algorithm is included as well in the folder `./cpp`. Compile using your favorite compiler:
```shell
$ cd ./cpp
$ clang++ -O3 -std=c++17 main.cpp -o main
```
To launch:
```shell
$ ./main
```
### Usage

To use this algorithm in your project, the following is required:
* Create a new mesh for a specific obj file, the `y_up` flag should be set to swap `y` and `z`, since the algorithm assumes `z_up`.
```rust
let mesh = Mesh::new(file_path, y_up);
```
* Traverse the mesh for a given point with x/y coordinate
```rust
let mut p = Vertex { x: 0.0, y: 0.0, z: 0.0 };
p.z = mesh.traverse(&p);
```
**Note**: only the `p.x` and `p.y` are used as input and `p.z` is ignored.

## Remarks

### CPU vs. GPU
This algorithm is designed to be portable and runs on the CPU. It can probably be done much faster on a GPU, but it can run in conjunction with a dynamic simulation (i.e. vehicle driving on a road).

### Hit Accuracy
When testing each triangle centroid with the included mesh, there will not be an identical result with the included mesh. This is because the included mesh has several vertical walls (guard rails) and the algorithm is designed for terrain lookups. In order to prevent this, these walls should be ignored/eliminated from the mesh in a pre-processing stage based on the direction of the face normal, but this is not yet implemented.

### Parallel Processing
It is possible to perform multiple traversals in parallel using i.e. the `rayon` crate, this can be faster in certain cases (6x speed-up when testing each triangle on my system), but it can also be slower if we only need to evaluate a small batch each time (i.e. 5 point lookup per wheel for a vehicle drivign on the terrain). In the case of small batches, the overhead in starting the threads is probably not worth it. To evaluate this in parallel, add the `rayon` package:
```shell
$ cargo add rayon
```
Update the `main.rs` file to include the `rayon` package
```rust
use rayon::prelude::*;
```
And change the `.iter()` method into a `.par_iter()`.


