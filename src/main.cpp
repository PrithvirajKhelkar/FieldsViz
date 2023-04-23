#include <vector>
#include <tuple>
#include <map>
#include <iostream>
#include <fstream>
#include <cmath>
#include <set>
#include <queue>
#include <sstream>
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace std;

polyscope::SurfaceMesh *psMesh;


int main(int argc, char **argv) {

  std::string filename = argv[1];
  std::ifstream in(filename);

  std::vector<std::vector<double>> positions;
  std::vector<std::vector<int>> faces;
  std::vector<std::vector<double>> field;
  string line;
  while( getline( in, line ))
      {
         std::stringstream ss( line );
         string token;

         ss >> token;

         if( token == "v"  ) {
            
            double x, y, z;
            ss >> x >> y >> z;
            std::vector<double> vec{x, y, z};
            positions.push_back(vec);
         };
         if( token == "f"  ) {
            int v1, v2, v3;
            ss >> v1 >> v2 >> v3;
            std::vector<int> face{v1-1, v2-1, v3-1};
            faces.push_back(face);
         };
         if( token == "vf"  ) {
            double x, y, z;
            ss >> x >> y >> z;
            std::vector<double> vec{x, y, z};
            field.push_back(vec);
         };
      }

  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("mesh", positions, faces);

  polyscope::getSurfaceMesh("mesh")->addVertexVectorQuantity("my_vector", field);
  polyscope::show();

  return 0;
}
