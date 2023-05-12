cd deps/tcods
make -j8
cd ..
cd ..
cd deps/fieldgen
make -j8 commandline
cd ..
cd ..
mkdir build
cd build
cmake ..
make -j8