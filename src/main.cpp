#include <vector>
#include <tuple>
#include <map>
#include <iostream>
#include <fstream>
#include <cmath>
#include <set>
#include <queue>
#include <sstream>
#include <string>
#include "polyscope/polyscope.h"
#include "polyscope/options.h"
#include "polyscope/pick.h"
#include "polyscope/render/engine.h"
#include "polyscope/view.h"
#include "polyscope/surface_mesh.h"
#include "imgui.h"
#include <algorithm>
#include "polyscope/point_cloud.h"

using namespace std;

string input_file_path;
polyscope::SurfaceMesh *psMesh;
polyscope::PointCloud* psCloud;

std::map<int, std::vector<double>> vertexMap;
std::vector<std::vector<double>> positions;
std::vector<std::vector<int>> faces;
std::vector<std::vector<double>> field;

std::vector<std::vector<double>> newield;

int lastClickedVertex;

std::vector<int> selectedVertices;
std::map<int, double> selectedVertexIndexMap;
int n_rings;
double angle;

bool curvatureAligned = false;
bool boundaryAligned = false;
bool alignmentAligned = false;

int n = 1;
double s=0;
double t=0;

bool alignments_generated = false;

void readEOBJ_FaceAttrs(string path, string fieldName) {
   std::ifstream in(path);
   string line;
   int i  = 0;
   while (getline(in, line))
   {
      std::stringstream ss(line);
      string token;

      ss >> token;
      if (token == "#attrsf")
      {
         double x, y, z;
         ss >> x >> y >> z;
         std::vector<double> vec{x, y, z};
         field.push_back(vec);
      };
   }
   psMesh = polyscope::registerSurfaceMesh("mesh", positions, faces);
   polyscope::getSurfaceMesh("mesh")->addFaceVectorQuantity(fieldName, field);
}

void readOBJ_Fields(string path, string fieldName) {
   std::ifstream in(path);
   string line;
   int i  = 0;
   while (getline(in, line))
   {
      std::stringstream ss(line);
      string token;

      ss >> token;
      if (token == "#attrsf")
      {
         double x, y, z;
         ss >> x >> y >> z;
         std::vector<double> vec{x, y, z};
         field.push_back(vec);
      };
   }
   psMesh = polyscope::registerSurfaceMesh("mesh", positions, faces);
   polyscope::getSurfaceMesh("mesh")->addFaceVectorQuantity(fieldName, field);
}


void run_tcods(){
   ofstream myfile;
   myfile.open ("../test/tcods_input.txt");
   myfile << "in " << input_file_path << "\n";
   myfile << "out ../test/temp_obj.eobj \n";
   for (int i = 0; i < selectedVertices.size(); i++) {
      int index = selectedVertices[i];
      myfile << "vertex " << index << " " << selectedVertexIndexMap[index] <<"\n";
   }
   myfile << "angle " << angle << "\n";
   myfile << "n_rings " << n_rings << "\n";
   myfile.close();

   system("../run_tcods.sh");

   readEOBJ_FaceAttrs("../test/temp_obj.eobj", "alignment fields");
   alignments_generated = true;
}

void run_fieldgen(){
   string args = "../run_fieldgen.sh " + std::to_string(n)+" ";
   if(curvatureAligned==true){
      args += "--alignToCurvature ";
   } else {
      args += "''";
   }
   if(boundaryAligned==true){
      args += "--alignToBoundary ";
   } else {
      args += "''";
   }
   if(alignmentAligned==true){
      args += "''";
   } else {
      args += "''";
   }
   args += "--s="+std::to_string(s)+" ";
   args += "--t="+std::to_string(t);
   system(args.c_str());

   readOBJ_Fields("../test/final_fields.obj", "vector fields");
}

std::vector<std::vector<double>> getSingularitiesList() {
   std::vector<std::vector<double>> singularities;
   for (int i = 0; i < selectedVertices.size(); i++) {
      int index = selectedVertices[i];
      singularities.push_back(vertexMap[index]);
   }
   return singularities;
}

bool removeIndexFromSelectedVertices(int index) {
   std::vector<int>::iterator it = std::find(selectedVertices.begin(), selectedVertices.end(), index);

   // if it is then remove it.
   if (it != selectedVertices.end()) {
      selectedVertices.erase(it);
      selectedVertexIndexMap[index] = 0.0;
      return true;
   } else {
      return false;
   }
}

void vertexClickEvent(int index) {
   if (lastClickedVertex != index && index < positions.size()){
      lastClickedVertex = index;

      // check if index is already selected
      std::vector<int>::iterator it = std::find(selectedVertices.begin(), selectedVertices.end(), index);

      // if it is then remove it.
      if (!removeIndexFromSelectedVertices(index)) {
        selectedVertices.push_back(index);
        selectedVertexIndexMap[index] = 0;
      }

      psCloud = polyscope::registerPointCloud("Singularities", getSingularitiesList());
      // set some options
      psCloud->setPointRadius(0.005);
   }
}

float imguiStackMargin = 10;
float lastWindowHeightPolyscope = 200;
float lastWindowHeightUser = 200;
float leftWindowsWidth = 305;
float rightWindowsWidth = 500;



void polyscope::buildPickGui() {
  if (pick::haveSelection()) {

    ImGui::SetNextWindowPos(ImVec2(view::windowWidth - (rightWindowsWidth + imguiStackMargin),
                                   2 * imguiStackMargin + lastWindowHeightUser));
    ImGui::SetNextWindowSize(ImVec2(rightWindowsWidth, 0.));

    ImGui::Begin("Selection", nullptr);
    std::pair<Structure*, size_t> selection = pick::getSelection();

    ImGui::TextUnformatted((selection.first->typeName() + ": " + selection.first->name).c_str());
    ImGui::Separator();
    selection.first->buildPickUI(selection.second);

    vertexClickEvent(selection.second);
    rightWindowsWidth = ImGui::GetWindowWidth();
    ImGui::End();
  }
}

void polyscope::PointCloud::buildCustomUI() {
  ImGui::Text("# points: %lld", static_cast<long long int>(nPoints()));
  if (ImGui::ColorEdit3("Point color", &pointColor.get()[0], ImGuiColorEditFlags_NoInputs)) {
    setPointColor(getPointColor());
  }
  ImGui::SameLine();
  ImGui::PushItemWidth(70);
  if (ImGui::SliderFloat("Radius", pointRadius.get().getValuePtr(), 0.0, .1, "%.5f",
                         ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_NoRoundToFormat)) {
    pointRadius.manuallyChanged();
    requestRedraw();
  }
  ImGui::PopItemWidth();

   for (int i = 0; i < selectedVertices.size(); i++) {
         int index = selectedVertices[i];
         ImGui::TextUnformatted(std::to_string(index).c_str());
         double& val = selectedVertexIndexMap[index];
         ImGui::InputDouble("Index", &val);
         ImGui::Separator();
   }

   ImGui::Separator();

   ImGui::InputDouble("Angle", &angle);
   ImGui::InputInt("N Rings", &n_rings);

   if (ImGui::Button("Calculate Alignment Fields")){
      // int status = system("../run_tcods.sh");
      run_tcods();
   }

   if (alignments_generated) {
      ImGui::InputInt("n", &n);

      ImGui::Checkbox("Curvature Aligned", &curvatureAligned);
      ImGui::Checkbox("Boundary Aligned", &boundaryAligned);
      ImGui::Checkbox("User Field Aligned", &alignmentAligned);

      ImGui::InputDouble("s", &s);
      ImGui::InputDouble("t", &t);
      if (ImGui::Button("Calculate Fields")){
         run_fieldgen();
      }
   }
}

void readOBJ() {
   std::ifstream in(input_file_path);
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
   }
   psMesh = polyscope::registerSurfaceMesh("mesh", positions, faces);
}





int main(int argc, char **argv)
{

   std::string filename = argv[1];
   input_file_path = filename;
   polyscope::init();
   readOBJ();

   polyscope::show();

   return 0;
}
