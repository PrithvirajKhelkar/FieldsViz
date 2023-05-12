# Visualizing vector fields over manifold meshes

## Overview
Visualize vector fields from [this paper](https://www.cs.cmu.edu/~kmcrane/Projects/GloballyOptimalDirectionFields/), with added alignment adapted from [this paper](https://www.cs.cmu.edu/~kmcrane/Projects/TrivialConnections/).
(This code has only been tested in WSL Ubuntu environment)

## Git
```
git clone --recursive git@github.com:PrithvirajKhelkar/FieldsViz.git
```
Don't forget the `recursive` keyword. We need three submodules (Polyscope, Tcods, Fieldgen).

Or if you forgot to use the `recursive` keyword, follow up with the following:
```
git submodule update --init --recursive
```

(expand on the overview soon)

## Build Instructions
### Dependencies
`suitesparse`, `metis`, `blas`, `lapack`
should be available at the default paths (`/usr/local/`)

### Building
Run the following commands from the project root directory:
```
make -C deps/tcods
make -C deps/fieldgen
mkdir build
cd build
cmake ..
make -j8
```
This builds fieldgen and tcods, as well as polyscope with fieldviz.

## Usage
```
./bin/fieldviz ../test/bunny.obj
```


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
* handles polyscope vis.
----------------------------------------------------------------------------------------------

Created as a part of the final project submission for *CAS CS 582: Geometry Processing (Spring 2023)* course, taught by *Prof. Edward Chien* at Boston University, Graduate School of Arts and Sciences.

