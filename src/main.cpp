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

// CONSTANTS

string SMOOTHEST = "smoothest";
string CURVATURE = "curvature";
string BOUNDARY = "boundary";
string ALIGNMENT = "alignment";

float imguiStackMargin = 10;
float lastWindowHeightPolyscope = 200;
float lastWindowHeightUser = 200;
float leftWindowsWidth = 305;
float rightWindowsWidth = 500;

//---------------

// Glocal variables
string input_file_path;  // variable for mesh file path
polyscope::SurfaceMesh *psMesh; // variable for polyscope mesh
polyscope::PointCloud* psCloud; // singularities as point clouds

std::map<int, std::vector<double>> vertexMap; // index and vertex map
std::vector<std::vector<double>> positions; // vertices 
std::vector<std::vector<int>> faces; // faces
std::vector<std::vector<double>> field; // fields

std::map<string, std::vector<std::vector<double>>> vectorFields;
std::map<string, std::map<int, double>> vectorSingularities;

int lastClickedVertex; // variable just to have the recently clicked vertex

std::vector<int> selectedVertices; // selected vertex. requested singularities
std::map<int, double> selectedVertexIndexMap; // indices
int n_rings;
double angle;

int counter = 0; // number of fields being shown/

string mode = SMOOTHEST;

int n = 1; //n-vectors
double s=0; //smoothness
double t=0; // lambda controller

bool alignments_generated = false;


void showSingularities(string fieldName = "") {
   """
   Function to show singularities by fieldName.
   """
   std::map<int, double> temp;

   if (fieldName == "") {
      temp = selectedVertexIndexMap;
   } else {
      temp = vectorSingularities[fieldName];
   }

   std::vector<std::vector<double>> singularities;
   std::vector<double> indices;

   for (const auto& key : temp) {
      int v = key.first;
      double index = key.second;
      singularities.push_back(vertexMap[v]);
      indices.push_back(index);
   }

   psCloud = polyscope::registerPointCloud((fieldName+"_singularities").c_str(), singularities);
   polyscope::getPointCloud((fieldName+"_singularities").c_str())->addScalarQuantity("holonomy", indices);
   psCloud->setPointRadius(0.005);
}


void readEOBJ_FaceAttrs(string path, string fieldName) {
   """
   Read Face vectors
   """
   field.clear();
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
   polyscope::getSurfaceMesh("mesh")->addFaceVectorQuantity(fieldName, field);
}

void readOBJ_Fields(string path, string fieldName) {
   """
   React vertex vectors
   """
   // vectorFields[fieldName].clear();
   // vectorSingularities[fieldName].clear();
   string name = (std::to_string(counter)+fieldName).c_str();
   std::ifstream in(path);
   string line;
   int i  = 0;
   double degree = 1;
   std::vector<std::vector<double>> normals;
   while (getline(in, line))
   {
      std::stringstream ss(line);
      string token;

      ss >> token;
      if (token == "#degree")
      {
         ss >> degree;
      }
      if (token == "vn")
      {
         double x, y, z;
         ss >> x >> y >> z;
         std::vector<double> vec{x, y, z};
         normals.push_back(vec);
      }
      if (token == "#field")
      {
         int i;
         double x, y, z;
         ss >> i >> x >> y >> z;
         std::vector<double> vec0{x, y, z};
         std::vector<double> vec1{-x, -y, -z};
         std::vector<double> vec2{x, y, z};
         std::vector<double> vec3{x, y, z};
         vectorFields[name].push_back(vec0);
         if (degree == 2 || degree == 4)
         {
            vectorFields[name+"1"].push_back(vec1);
         }
         if (degree == 4)
         {
            int curInd = vectorFields[name].size()-1;
            vec2[0] = vectorFields[name][curInd][1]*normals[curInd][2] - vectorFields[name][curInd][2]*normals[curInd][1];
            vec2[1] = vectorFields[name][curInd][2]*normals[curInd][0] - vectorFields[name][curInd][0]*normals[curInd][2];
            vec2[2] = vectorFields[name][curInd][0]*normals[curInd][1] - vectorFields[name][curInd][1]*normals[curInd][0];
            vec3[0] = -vec2[0];
            vec3[1] = -vec2[1];
            vec3[2] = -vec2[2];
            vectorFields[name+"2"].push_back(vec2);
            vectorFields[name+"3"].push_back(vec3);
         }
      }
      if(token == "#singularity") {
         int i;
         double j;
         ss >> i >> j;
         vectorSingularities[name][faces[i-1][0]] = j;
         
      }
   }
   polyscope::getSurfaceMesh("mesh")->addVertexVectorQuantity(name, vectorFields[name]);
   if(degree == 2 || degree == 4){
      polyscope::getSurfaceMesh("mesh")->addVertexVectorQuantity(name+"1", vectorFields[name+"1"]);
   }
   if (degree == 4){
      polyscope::getSurfaceMesh("mesh")->addVertexVectorQuantity(name+"2", vectorFields[name+"2"]);
      polyscope::getSurfaceMesh("mesh")->addVertexVectorQuantity(name+"3", vectorFields[name+"3"]);
   }
   showSingularities(name);
   
   counter++;
}


void run_tcods(){
   """
   Function to run trivial connections script
   """
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
   """
   Function to run Fieldgen connections script
   """
   string args = "../run_fieldgen.sh " + std::to_string(n)+" ";
   string name = "smoothest";

   if (mode == SMOOTHEST) {
      args += "''";
      name = "smoothest";
   }
   else if(mode == CURVATURE){
      args += "--alignToCurvature ";
      name = "curvature_aligned";
   } 
   else if(mode == BOUNDARY){
      args += "--alignToBoundary ";
      name = "boundary_aligned";
   } else if(mode == ALIGNMENT){
      args += "--alignToGivenField";
      name = "alignment_aligned";
   } else {
      args += "''";
   }
   args += " "+std::to_string(s);
   args += " "+std::to_string(t);
   system(args.c_str());

   readOBJ_Fields("../test/final_fields.obj", name);
}



bool removeIndexFromSelectedVertices(int index) {
   """
   Function to remove a selected singularity
   """
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
   """
   Function to handle vertex clicks.
   Adds vertex to singularity list.
   """
   if (lastClickedVertex != index && index < positions.size()){
      lastClickedVertex = index;

      // check if index is already selected
      std::vector<int>::iterator it = std::find(selectedVertices.begin(), selectedVertices.end(), index);

      // if it is then remove it.
      if (!removeIndexFromSelectedVertices(index)) {
        selectedVertices.push_back(index);
        selectedVertexIndexMap[index] = 0.0;
      }

      showSingularities("");
   }
}





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
    if(selection.first->name == "mesh"){vertexClickEvent(selection.second);}
    
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
  if(name=="_singularities"){
      for (int i = 0; i < selectedVertices.size(); ++i) {
            int index = selectedVertices[i];
            ImGui::TextUnformatted(std::to_string(index).c_str());
            ImGui::InputDouble(("Index_"+std::to_string(index)).c_str(), &selectedVertexIndexMap[index]);
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
         string align = " align";
         if(ImGui::RadioButton((SMOOTHEST+align).c_str(), mode == SMOOTHEST)){
            mode = SMOOTHEST;
         };
         // if(ImGui::RadioButton((CURVATURE+align).c_str(), mode == CURVATURE)){
         //    mode = CURVATURE;
         // };
         // if(ImGui::RadioButton((BOUNDARY+align).c_str(), mode == BOUNDARY)){
         //    mode = BOUNDARY;
         // };
         if(ImGui::RadioButton((ALIGNMENT+align).c_str(), mode == ALIGNMENT)){
            mode = ALIGNMENT;
         };

         ImGui::InputDouble("s", &s);
         ImGui::InputDouble("t", &t);
         if (ImGui::Button("Calculate Fields")){
            run_fieldgen();
         }
      }
  }
}

void readOBJ() {
   """
   Read the input obj file
   """
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
