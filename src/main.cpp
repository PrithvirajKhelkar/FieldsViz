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
#include "imgui.h"
#include <algorithm>

using namespace std;

polyscope::SurfaceMesh *psMesh;

std::map<int, std::vector<double>> vertexMap;


int lastClickedVertex;

std::vector<int> selectedVertices;
std::map<int, double> selectedVertexIndexMap;

bool removeIndexFromSelectedVertices(int index) {
   std::vector<int>::iterator it = std::find(selectedVertices.begin(), selectedVertices.end(), index);

   // if it is then remove it.
   if (it != selectedVertices.end()) {
      selectedVertices.erase(it);
      selectedVertexIndexMap[index] = 0;
      return true;
   } else {
      return false;
   }
}

void vertexClickEvent(int index) {
   if (lastClickedVertex != index){
      lastClickedVertex = index;

      // check if index is already selected
      std::vector<int>::iterator it = std::find(selectedVertices.begin(), selectedVertices.end(), index);

      // if it is then remove it.
      if (!removeIndexFromSelectedVertices(index)) {
        selectedVertices.push_back(index);
        selectedVertexIndexMap[index] = 0;
      }
   }
}


void polyscope::SurfaceMesh::buildPickUI(size_t localPickID) {
   // Selection type
  if (localPickID < facePickIndStart) {
   polyscope::SurfaceMesh::buildVertexInfoGui(localPickID);
   vertexClickEvent(localPickID);
   // for (int x : selectedVertices) {
   //       polyscope::SurfaceMesh::buildVertexInfoGui(x);
   // }
  } else if (localPickID < edgePickIndStart) {
    polyscope::SurfaceMesh::buildFaceInfoGui(localPickID - facePickIndStart);
  } else if (localPickID < halfedgePickIndStart) {
    polyscope::SurfaceMesh::buildEdgeInfoGui(localPickID - edgePickIndStart);
  } else {
    polyscope::SurfaceMesh::buildHalfedgeInfoGui(localPickID - halfedgePickIndStart);
  }
}



int main(int argc, char **argv)
{

   std::string filename = argv[1];
   std::ifstream in(filename);

   std::vector<std::vector<double>> positions;
   std::vector<std::vector<int>> faces;
   std::vector<std::vector<double>> field;
   string line;

   int i  = 0;
   while (getline(in, line))
   {
      std::stringstream ss(line);
      string token;

      ss >> token;

      if (token == "v")
      {

         double x, y, z;
         ss >> x >> y >> z;
         std::vector<double> vec{x, y, z};
         positions.push_back(vec);
         vertexMap[i] = vec;

         i++;
      };
      if (token == "f")
      {
         int v1, v2, v3;
         ss >> v1 >> v2 >> v3;
         std::vector<int> face{v1 - 1, v2 - 1, v3 - 1};
         faces.push_back(face);
      };
      if (token == "vf")
      {
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
