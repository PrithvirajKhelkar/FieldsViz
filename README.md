# Visualizing vector fields over manifold meshes

## Overview
Visualize vector fields from [this paper](https://www.cs.cmu.edu/~kmcrane/Projects/GloballyOptimalDirectionFields/), with added alignment adapted from [this paper](https://www.cs.cmu.edu/~kmcrane/Projects/TrivialConnections/).
(This code has only been tested in WSL Ubuntu environment)


(expand on the overview soon)

## Build Instructions
### Dependencies
`suitesparse`, `metis`, `blas`, `lapack`
files should be available at the default paths (`/usr/local/lib` and `/usr/local/include`)

### Building
IMPORTANT. UPDATE THE SUBMODULES. CODE WON'T RUN WITHOUT IT.:
```
git submodule update --init --recursive
```
Then run the following commands from project root directory.
```
make -C deps/tcods
make -C deps/fieldgen
mkdir build
cd build
cmake .
make -j8
```
This builds fieldgen and tcods, as well as polyscope with fieldviz.

## Usage
```
./bin/fieldviz ../test/bunny.obj
```

* Click on any vertex to select it as a singularity.
* Set the Indices in the singularities UI on the left.
* Click on `create alignment fields`
* Choose between smooth or aligned fields.
* Set values of `s`, `t`.


## Important Files
### TCODS:
* `MeshIO.cpp::writeEOBJ` which has the `BFS` code.
* `HalfEdge.cpp::toGlobal` where we removed origin so that the face based vectors made more sense which weren't lying on the faces before.

### Fieldgen:
* `src/KVecDir.cpp`: `SmoothestGivenVectorAlignment`, `setupqForGivenVectorAlignment`, `ComputeInputVectorFields`
* `src/commandline.cpp` and `src/viewer.cpp` to take inputs
* And other mesh files like `MeshIO`, `Mesh.h` to store the alignment fields and to read and write the in proper format

### Main.cpp
* main file to read mesh.
* handles polyscope visualizations.
----------------------------------------------------------------------------------------------

Created as a part of the final project submission for *CAS CS 582: Geometry Processing (Spring 2023)* course, taught by *Prof. Edward Chien* at Boston University, Graduate School of Arts and Sciences.

