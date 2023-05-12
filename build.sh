#!/bin/bash
make -j8 -C deps/tcods 
make -j8 commandline  -C deps/fieldgen/
mkdir build
cd build
cmake ..
make -j8