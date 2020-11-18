/**
	* @file OpenHSML.cpp
  * @brief
  *
	* @author Jordan Nowak
	* @version 1.0.0
	* @date 31-10-2020
	*/
#include "OpenHSML.h"

#ifndef OPENHSML
#define OPENHSML

bool finished;
bool finished2d;
bool finished3d;

static void mouse_call(int event,int x,int y, int, void* param) {
  if(event==cv::EVENT_LBUTTONDOWN){
    std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
    ptPtr->push_back(cv::Point(x,y));
    finished=true;
  }
}
static void mouse_call2d(int event,int x,int y, int, void* param) {
  if(event==cv::EVENT_LBUTTONDOWN){
    std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
    ptPtr->push_back(cv::Point(x,y));
    finished2d=true;
  }
}
static void mouse_call3d(int event,int x,int y, int, void* param) {
  if(event==cv::EVENT_LBUTTONDOWN){
    std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
    ptPtr->push_back(cv::Point(x,y));
    finished3d=true;
  }
}
std::vector<std::string> split(const std::string& s, char delimiter) {
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter)) { tokens.push_back(token); }
   return tokens;
}

// Reading:
void OpenHSML::convert_yaml_file_to_cvmat(std::string yml_img_namefile, cv::Mat& img) {
  cv::FileStorage fsr(yml_img_namefile, cv::FileStorage::READ);
  fsr["data"] >> img;
  fsr.release();
}
void OpenHSML::convert_cvmat_to_vector(cv::Mat img, std::vector<double>& myVec) {
  myVec.assign( (double*)img.datastart, (double*)img.dataend );
}
void OpenHSML::convert_yaml_file_to_vector(std::string yml_img_namefile, std::vector<double>& myVec) {
  cv::Mat myImg;
  convert_yaml_file_to_cvmat(yml_img_namefile, myImg);
  convert_cvmat_to_vector(myImg, myVec);
}
void OpenHSML::convert_vector_to_xyz_vector(std::vector<double> vect, std::string calibration_namefile, std::vector<float>& x_vect, std::vector<float>& y_vect, std::vector<float>& z_vect) {
  std::vector<double> depth_parameters;
  read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);

  if ( (vect.size())==(depth_parameters[0] * depth_parameters[1]) ) {
    float fu=(depth_parameters[0]/2)/tan(depth_parameters[2]/2* M_PI /180);
    float fv=(depth_parameters[1]/2)/tan(depth_parameters[3]/2* M_PI /180);

    x_vect.clear(); y_vect.clear(); z_vect.clear();
    for (size_t j=0; j < (vect.size()); j++) {
      int v = (int)(j/((int)depth_parameters[0])), u = (int)(j%((int)depth_parameters[0])-1);
      z_vect.push_back( vect[j] );
      x_vect.push_back( vect[j] * (u-depth_parameters[4])/fu );
      y_vect.push_back( vect[j] * (v-depth_parameters[5])/fv );
    }
  }
  else if ( (vect.size())==3*(depth_parameters[0] * depth_parameters[1]) ) {
    x_vect.clear(); y_vect.clear(); z_vect.clear();
    for (size_t j=0; j < (vect.size()); j=j+3) {
      x_vect.push_back( vect[j] );
      y_vect.push_back( vect[j+1] );
      z_vect.push_back( vect[j+2] );
    }
  }
  else {
    std::cout << "ERROR: size between 2D image and depth value not compatible !" << '\n';
    std::cout << "   - vect.size()   = " << vect.size() << '\n';
    std::cout << "   - img_3D.size() = " << (depth_parameters[0] * depth_parameters[1]) << '\n';
    exit(0);
  }
}
void OpenHSML::convert_depth_vector_to_img_grayscale(std::vector<float> z_vect, cv::Mat& ImgGrayScale, int imageWidth, int imageHeight) {
  std::vector<int> z_array_norm(imageWidth*imageHeight);
  const auto [min, max] = std::minmax_element(begin(z_vect), end(z_vect));
  for (size_t j = 0; j < z_vect.size(); j++) {
    if ( (0.<(float)z_vect[j]) & ((float)z_vect[j]<=3.) ) { z_array_norm[j] = (int)((float)z_vect[j] * 200. / 3.) + 0; }
    else if ( (3.<(float)z_vect[j]) & ((float)z_vect[j]<=6.) ) { z_array_norm[j] = (int)((float)z_vect[j] * 40.  / 6.) + 200; }
    else if ( (6.<(float)z_vect[j]) & ((float)z_vect[j]<=9.) ) { z_array_norm[j] = (int)((float)z_vect[j] * 15.  / 9.) + 240 ; }
    else { z_array_norm[j] = 0; }
  }

  std::vector<uint8_t> newVec;
  for (size_t i=0; i<z_array_norm.size(); i++) {
    newVec.push_back( z_array_norm[i] );
    newVec.push_back( z_array_norm[i] );
    newVec.push_back( z_array_norm[i] );
  }

  ImgGrayScale = cv::Mat(imageHeight, imageWidth, CV_8UC3);
  memcpy(ImgGrayScale.data,newVec.data(),newVec.size()*sizeof(uint8_t));
}

OpenHSML::OpenHSML() {};
OpenHSML::~OpenHSML() {};

// Calibration:
void OpenHSML::read_yaml_file_of_calibration(std::string calibration_namefile, std::vector<double>& depth_parameters) {
  YAML::Node config = YAML::LoadFile( calibration_namefile );
  const YAML::Node& calibration = config["calibration"];

  const YAML::Node& fundamental_matrix_node = calibration["fundamental_matrix"];
  const YAML::Node& fundamental_matrix_rows = fundamental_matrix_node["rows"];
  const YAML::Node& fundamental_matrix_cols = fundamental_matrix_node["cols"];
  const YAML::Node& fundamental_matrix_matx = fundamental_matrix_node["matrix"];

  const YAML::Node& projection_matrix_node = calibration["projection_matrix"];
  const YAML::Node& projection_matrix_rows = projection_matrix_node["rows"];
  const YAML::Node& projection_matrix_cols = projection_matrix_node["cols"];
  const YAML::Node& projection_matrix_matx = projection_matrix_node["matrix"];

  const YAML::Node& depth_image = calibration["depth_image"];
  const YAML::Node& depth_width = depth_image["width"];
  const YAML::Node& depth_height = depth_image["height"];
  const YAML::Node& HFOV = depth_image["HFOV"], VFOV = depth_image["VFOV"];
  const YAML::Node& cu = depth_image["cu"], cv = depth_image["cv"];

  std::vector<double> F_vec, P_vec;
  F_vec.clear(); P_vec.clear();

  Eigen::Matrix<double, 3, 4> P;
  P << projection_matrix_matx[0].as<double>(), projection_matrix_matx[1].as<double>(), projection_matrix_matx[2].as<double>(), projection_matrix_matx[3].as<double>(),
    projection_matrix_matx[4].as<double>(), projection_matrix_matx[5].as<double>(), projection_matrix_matx[6].as<double>(), projection_matrix_matx[7].as<double>(),
    projection_matrix_matx[8].as<double>(), projection_matrix_matx[9].as<double>(), projection_matrix_matx[10].as<double>(), projection_matrix_matx[11].as<double>();
  projection_matrix = P;

  std::vector<double> myVec;
  myVec.assign((double*)fundamental_matrix.datastart, (double*)fundamental_matrix.dataend);

  for (size_t j = 0; j < fundamental_matrix_matx.size(); j++) { F_vec.push_back(fundamental_matrix_matx[j].as<double>()); }
  cv::Mat newImg_f=cv::Mat(3, 3, CV_64F);
  memcpy(newImg_f.data,F_vec.data(),F_vec.size()*sizeof(double));
  newImg_f.convertTo(fundamental_matrix, CV_64F);

  depth_parameters.clear();
  depth_parameters.push_back(depth_width.as<double>()); depth_parameters.push_back(depth_height.as<double>());
  depth_parameters.push_back(HFOV.as<double>()); depth_parameters.push_back(VFOV.as<double>());
  depth_parameters.push_back(cu.as<double>()); depth_parameters.push_back(cv.as<double>());

  std::cout << "     \x1B[4m" << "Intrinsic parameters saved of the depth camera:" << "\n\x1B[0m";
  std::cout << "       -> Depth image size        : width  = " << ( depth_parameters[0] ) << std::endl;
  std::cout << "       -> Depth image size        : height = " << ( depth_parameters[1] ) << std::endl;
  std::cout << "       -> Horizontal Field Of View: HFOV   = " << ( depth_parameters[2] ) << std::endl;
  std::cout << "       -> Vertical Field Of View  : VFOV   = " << ( depth_parameters[3] ) << std::endl;
  std::cout << "       -> Optical center u        : cu     = " << ( depth_parameters[4] ) << std::endl;
  std::cout << "       -> Optical center v        : cv     = " << ( depth_parameters[5] ) << std::endl << std::endl;
}
void OpenHSML::read_yaml_file_of_point_of_calibration(std::string calibration_namefile, std::vector<cv::Point2d>& points_3D, std::vector<cv::Point2d>& points_2D, std::vector<std::array<float, 3>>& points_xyz) {
  YAML::Node config = YAML::LoadFile( calibration_namefile );
  const YAML::Node& u_2D = config["u_2D"], v_2D = config["v_2D"];
  const YAML::Node& u_3D = config["u_3D"], v_3D = config["v_3D"];
  const YAML::Node& x_3D = config["x_3D"], y_3D = config["y_3D"], z_3D = config["z_3D"];

  std::vector<cv::Point2d> points_3D_, points_2D_;
  points_2D_.clear(); points_3D_.clear();
  for (size_t j = 0; j < u_2D.size(); j++) {
    if (  u_2D.size()==v_2D.size() & u_2D.size()==u_3D.size() & u_3D.size()==v_3D.size() ) {
      points_2D_.push_back(cv::Point2d(u_2D[j].as<double>(), v_2D[j].as<double>()));
      points_3D_.push_back(cv::Point2d(u_3D[j].as<double>(), v_3D[j].as<double>()));
    }
  }

  points_2D.clear(), points_3D.clear(); points_xyz.clear();
  for (size_t i=0; i<points_2D_.size(); i++) {
    points_3D.push_back(cv::Point2d(points_3D_[i].x, points_3D_[i].y));
    points_2D.push_back(cv::Point2d(points_2D_[i].x, points_2D_[i].y));
    points_xyz.push_back({x_3D[i].as<double>(), y_3D[i].as<double>(), z_3D[i].as<double>()});
  }
}
void OpenHSML::modification_of_camera_parameters(std::vector<double>& depth_parameters) {
  std::string yes_no="";
  while (yes_no!="no" & yes_no!="n" & yes_no!="N") {
    while (yes_no!="yes" & yes_no!="no" & yes_no!="y" & yes_no!="n" & yes_no!="Y" & yes_no!="N") {
      std::cout<<"\x1B[2J\x1B[;H"<<std::endl; // clear all
      std::cout << "     \x1B[4m" << "Intrinsic parameters saved of the depth camera:" << "\n\x1B[0m";
      std::cout << "       -> Depth image size        : width  = " << ( depth_parameters[0] ) << std::endl;
      std::cout << "       -> Depth image size        : height = " << ( depth_parameters[1] ) << std::endl;
      std::cout << "       -> Horizontal Field Of View: HFOV   = " << ( depth_parameters[2] ) << std::endl;
      std::cout << "       -> Vertical Field Of View  : VFOV   = " << ( depth_parameters[3] ) << std::endl;
      std::cout << "       -> Optical center u        : cu     = " << ( depth_parameters[4] ) << std::endl;
      std::cout << "       -> Optical center v        : cv     = " << ( depth_parameters[5] ) << std::endl << std::endl;

      std::cout << "     \x1B[1m\x1B[4m"<< "Would you like to change this data? ('yes' or 'no')"<<"\x1B[0m"<<std::endl;
      std::cout << "     | If you choose 'yes', you will be able to change each value here." << std::endl;
      std::cout << "     | But if you prefer, select 'no' and you will be able to change   " << std::endl;
      std::cout << "     | this data manually in the following file:                       " << std::endl;
      std::cout << "     | '.../OpenHSML/share/resources/calibration/calibration.yaml'" << std::endl << std::endl;
      std::cout << "     Typing on a keyboard 'yes' or 'no'" << std::endl;
      std::cin >> yes_no;
    }
    if (yes_no=="yes" | yes_no=="y" | yes_no=="Y") {
      std::string input_str;
      std::cout<<"\x1B[2J\x1B[;H"<<std::endl; // clear all in terminal
      std::cout << "     \x1B[4m" << "Intrinsic parameters of the depth camera:" << "\n\x1B[0m";
      try { std::cout << "       -> Depth image size        : width  (currently: "<<depth_parameters[0]<<") = "; std::cin >> input_str; depth_parameters[0]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      try { std::cout << "       -> Depth image size        : height (currently: "<<depth_parameters[1]<<") = "; std::cin >> input_str; depth_parameters[1]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      try { std::cout << "       -> Horizontal Field Of View: HFOV (currently: "<<depth_parameters[2]<<") = "; std::cin >> input_str; depth_parameters[2]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      try { std::cout << "       -> Vertical Field Of View  : VFOV (currently: "<<depth_parameters[3]<<") = "; std::cin >> input_str; depth_parameters[3]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      try { std::cout << "       -> Optical center u        : cu (currently: "<<depth_parameters[4]<<") = "; std::cin >> input_str; depth_parameters[4]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      try { std::cout << "       -> Optical center v        : cv (currently: "<<depth_parameters[5]<<") = "; std::cin >> input_str; depth_parameters[5]=std::stod(input_str); }
      catch (const std::exception& e) { std::cerr << e.what() << std::endl; }
      yes_no="";
    }
  }
}
void OpenHSML::write_yaml_file_of_calibration(std::vector<double> depth_parameters, std::string calibration_namefile) {
  YAML::Node node_calibration_w, node_calibration;
  YAML::Node node_rgb_image, node_rgb_parameter;
  YAML::Node node_depth_image, node_depth_parameter;

  node_depth_parameter["width"] = YAML::Load(std::to_string(depth_parameters[0]));
  node_depth_parameter["height"] = YAML::Load(std::to_string(depth_parameters[1]));
  node_depth_parameter["HFOV"] = YAML::Load(std::to_string(depth_parameters[2]));
  node_depth_parameter["VFOV"] = YAML::Load(std::to_string(depth_parameters[3]));
  node_depth_parameter["cu"] = YAML::Load(std::to_string(depth_parameters[4]));
  node_depth_parameter["cv"] = YAML::Load(std::to_string(depth_parameters[5]));

  YAML::Node node_f, node_p;
  node_f["rows"] = YAML::Load("3"); node_p["rows"] = YAML::Load("3");
  node_f["cols"] = YAML::Load("3"); node_p["cols"] = YAML::Load("4");
  node_f["matrix"] = YAML::Load("[0, 0, 0, 0, 0, 0, 0, 0, 0]");
  node_p["matrix"] = YAML::Load("[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]");

  // fundamental_matrix to vect
  std::vector<double> myVec;
  myVec.assign((double*)fundamental_matrix.datastart, (double*)fundamental_matrix.dataend);

  node_f["matrix"] = YAML::Load("["+ std::to_string(myVec[0])+
                                ", "+std::to_string(myVec[1])+
                                ", "+std::to_string(myVec[2])+
                                ", "+std::to_string(myVec[3])+
                                ", "+std::to_string(myVec[4])+
                                ", "+std::to_string(myVec[5])+
                                ", "+std::to_string(myVec[6])+
                                ", "+std::to_string(myVec[7])+
                                ", "+std::to_string(myVec[8])+"]");

  node_p["matrix"] = YAML::Load("["+ std::to_string(projection_matrix(0,0))+
                                ", "+std::to_string(projection_matrix(0,1))+
                                ", "+std::to_string(projection_matrix(0,2))+
                                ", "+std::to_string(projection_matrix(0,3))+
                                ", "+std::to_string(projection_matrix(1,0))+
                                ", "+std::to_string(projection_matrix(1,1))+
                                ", "+std::to_string(projection_matrix(1,2))+
                                ", "+std::to_string(projection_matrix(1,3))+
                                ", "+std::to_string(projection_matrix(2,0))+
                                ", "+std::to_string(projection_matrix(2,1))+
                                ", "+std::to_string(projection_matrix(2,2))+
                                ", "+std::to_string(projection_matrix(2,3))+"]");

  node_calibration["depth_image"] = node_depth_parameter;
  node_calibration["fundamental_matrix"] = node_f;
  node_calibration["projection_matrix"] = node_p;
  node_calibration_w["calibration"] = node_calibration;

  std::ofstream outYaml;
  outYaml.open(calibration_namefile);
  outYaml << node_calibration_w;
  outYaml.close();
}
void OpenHSML::modification_of_calibration_of_points(std::vector<cv::Point2d>& points_3D_yaml_save, std::vector<cv::Point2d>& points_2D_yaml_save, std::vector<std::array<float, 3>>& points_xyz_yaml_save, std::vector<double>& depth_parameters, std::string calibration_namefile) {
  std::vector<cv::Point> points_2D, points_3D;
  std::vector<std::array<float, 3>> points_xyz;

  std::vector<std::string> name_file_2d, name_file_3d, name_file_xyz;
  for(auto& p: std::experimental::filesystem::directory_iterator(calibration_namefile+"/2d"))
  name_file_2d.push_back(p.path());
  std::sort(name_file_2d.begin(), name_file_2d.end());
  for(auto& p: std::experimental::filesystem::directory_iterator(calibration_namefile+"/depth")) // xyz ?
  name_file_xyz.push_back(p.path());
  std::sort(name_file_xyz.begin(), name_file_xyz.end());

  std::string yes_no="";
  while (yes_no!="yes" & yes_no!="no" & yes_no!="y" & yes_no!="n" & yes_no!="Y" & yes_no!="N") {
    std::cout<<"\x1B[2J\x1B[;H"<<std::endl; // clear all
    std::cout << "     \x1B[1m\x1B[4m"<< "Do you want to make point selection on the captured 2D and 3D images? ('yes' or 'no')"<<"\x1B[0m"<<std::endl;
    std::cout << "     | Before that, you have to put the images previously saved with your system in the folders:" << std::endl;
    std::cout << "     | '.../OpenHSML/share/resources/calibration/2d/'" << std::endl;
    std::cout << "     | '.../OpenHSML/share/resources/calibration/depth/'" << std::endl << std::endl;
    std::cout << "     Typing on a keyboard 'yes' or 'no'" << std::endl;
    std::cin >> yes_no;
  }
  if (yes_no=="yes" | yes_no=="y" | yes_no=="Y") {
    int i=0, k=0;
    points_3D_yaml_save.clear(); points_2D_yaml_save.clear(); points_xyz_yaml_save.clear();
    while (k<name_file_2d.size()) {
      std::vector<double> vect;
      std::vector<float> x, y, z;
      cv::Mat Image3D, Image2D = cv::imread(name_file_2d[k]);
      convert_yaml_file_to_vector(name_file_xyz[k].c_str(), vect);
      convert_vector_to_xyz_vector(vect, calibration_namefile, x, y, z);
      convert_depth_vector_to_img_grayscale(z, Image3D, depth_parameters[0], depth_parameters[1]);

      std::cout << "["<< k+1 <<"/"<< name_file_2d.size() <<"]" << std::endl;

      bool next_img=false, quadrilater_mode=false, line_mode=false, point_mode=true;
      while ( next_img==false ) {
      cv::namedWindow("Original2D"); cv::namedWindow("Original3D_without_selection");
      cv::setMouseCallback("Original2D", mouse_call2d, (void*)&points_2D);

      // normal:
      if ( (point_mode==true) & (quadrilater_mode==false) & (line_mode==false) ) {
        while( (!finished2d) & (quadrilater_mode==false) & (line_mode==false) ) {
          cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
          cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
          char keyboard = (char)cv::waitKey(50);
          if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false; break;}
          if(keyboard == 108) { std::cout << "'l' pressed: line mode activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false; break;}
          if(keyboard == 112) { std::cout << "'p' pressed: point mode already activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true;}
          if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; next_img=true; break;}
        }

        if ((next_img==false) & (point_mode==true)) {
          finished2d=false; finished3d=false;
          cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);

          cv::destroyWindow("Original2D"); cv::destroyWindow("Original3D_without_selection");
          cv::namedWindow("Original3D"); cv::namedWindow("Original2D_without_selection");
          cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
          while(!finished3d) {
            cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
            cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
            cv::waitKey(50);
          }

          finished2d=false; finished3d=false;
          cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
          cv::destroyWindow("Original3D"); cv::destroyWindow("Original2D_without_selection");

          points_2D_yaml_save.push_back( {(double)points_2D[i].x, (double)points_2D[i].y} );
          points_3D_yaml_save.push_back( {(double)points_3D[i].x, (double)points_3D[i].y} );
          points_xyz_yaml_save.push_back({x[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], y[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], z[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ]});

          std::cout << "===== Point " << i << " selected ====== " << std::endl;
          std::cout << "Point selected => 2D: " << points_2D_yaml_save[i].x << ", " << points_2D_yaml_save[i].y << std::endl;
          std::cout << "Point selected => 3D: " << points_3D_yaml_save[i].x << ", " << points_3D_yaml_save[i].y << std::endl;
          std::cout << "Point selected => xyz: " << points_xyz_yaml_save[i][0] << ", " << points_xyz_yaml_save[i][1] << ", " << points_xyz_yaml_save[i][2] << std::endl << std::endl;

          i=i+1;
        }
      }

      // quadrilater:
      if ( (point_mode==false) & (quadrilater_mode==true) & (line_mode==false) ) {
        while( (!finished2d) & (point_mode==false) & (line_mode==false) ) {
          cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
          cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
          char keyboard = (char)cv::waitKey(50);
          if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode already activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false;}
          if(keyboard == 108) { std::cout << "'l' pressed: line mode activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false; break;}
          if(keyboard == 112) { std::cout << "'p' pressed: point mode activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true; break;}
          if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; next_img=true; break;}
        }

        if ((next_img==false) & (quadrilater_mode==true)) {
          finished2d=false; finished3d=false;
          cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 1 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;

          // 2
          cv::setMouseCallback("Original2D", mouse_call2d, (void*)&points_2D);

          while( (!finished2d) ) {
            cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
            cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl;}
          }
          finished2d=false; finished3d=false;
          cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 2 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;

          // 3
          cv::setMouseCallback("Original2D", mouse_call2d, (void*)&points_2D);

          while( (!finished2d) ) {
            cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
            cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a third point." << std::endl;}
          }
          finished2d=false; finished3d=false;
          cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 3 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;

          // 4
          cv::setMouseCallback("Original2D", mouse_call2d, (void*)&points_2D);

          while( (!finished2d) ) {
            cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
            cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a fourth point." << std::endl; }
          }
          finished2d=false; finished3d=false;
          cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 4 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n';

          // Draw square
          int nb_of_pt_by_line = 8;
          std::vector<std::array<float, 2>> uv_vect; uv_vect.clear();
          sampling_square_with_four_points( {points_2D[i-3].x, points_2D[i-3].y}, {points_2D[i-2].x, points_2D[i-2].y}, {points_2D[i-1].x, points_2D[i-1].y}, {points_2D[i-0].x, points_2D[i-0].y}, nb_of_pt_by_line, uv_vect);
          for (size_t k = 0; k < uv_vect.size(); k++) {
            points_2D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) ); i=i+1;
            cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          }

          i=i-4-uv_vect.size()+1;

          // 3D points
          // 1
          finished2d=false; finished3d=false;
          cv::destroyWindow("Original2D"); cv::destroyWindow("Original3D_without_selection");
          cv::namedWindow("Original3D"); cv::namedWindow("Original2D_without_selection");
          cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
          while(!finished3d) {
            cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
            cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a first point." << std::endl; }
          }
          finished2d=false; finished3d=false;
          cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 1 selected in depth image: " << points_3D[i].x << ", " << points_3D[i].y << '\n'; i=i+1;

          // 2
          cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
          while(!finished3d) {
            cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
            cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl; }
          }
          finished2d=false; finished3d=false;
          cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 2 selected in depth image: " << points_3D[i].x << ", " << points_3D[i].y << '\n'; i=i+1;

          // 3
          cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
          while(!finished3d) {
            cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
            cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a third point." << std::endl; }
          }
          finished2d=false; finished3d=false;
          cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 3 selected in depth image: " << points_3D[i].x << ", " << points_3D[i].y << '\n'; i=i+1;

          // 4
          cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
          while(!finished3d) {
            cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
            cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
            if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a fourth point." << std::endl; }
          }
          finished2d=false; finished3d=false;
          cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point 4 selected in depth images: " << points_3D[i].x << ", " << points_3D[i].y << '\n';

          // Draw square
          uv_vect.clear();
          sampling_square_with_four_points( {points_3D[i-3].x, points_3D[i-3].y},
            {points_3D[i-2].x, points_3D[i-2].y},
            {points_3D[i-1].x, points_3D[i-1].y},
            {points_3D[i-0].x, points_3D[i-0].y},
            nb_of_pt_by_line, uv_vect);
            for (size_t k = 0; k < uv_vect.size(); k++) {
              points_3D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) ); i=i+1;
              cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
            }

            finished2d=false; finished3d=false;
            cv::destroyWindow("Original3D"); cv::destroyWindow("Original2D_without_selection");

            // save
            i=i-4-uv_vect.size()+1;
            for (size_t m = 0; m < (4+uv_vect.size()); m++) {
              points_2D_yaml_save.push_back( {(double)points_2D[i].x, (double)points_2D[i].y} );
              points_3D_yaml_save.push_back( {(double)points_3D[i].x, (double)points_3D[i].y} );
              points_xyz_yaml_save.push_back({x[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], y[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], z[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ]});

              std::cout << "===== Point " << i << " selected ====== " << std::endl;
              std::cout << "Point selected => 2D: " << points_2D_yaml_save[i].x << ", " << points_2D_yaml_save[i].y << std::endl;
              std::cout << "Point selected => 3D: " << points_3D_yaml_save[i].x << ", " << points_3D_yaml_save[i].y << std::endl;
              std::cout << "Point selected => xyz: " << points_xyz_yaml_save[i][0] << ", " << points_xyz_yaml_save[i][1] << ", " << points_xyz_yaml_save[i][2] << std::endl << std::endl;

              i=i+1;
            }
          }
        }

        // mode ligne:
        if ( (point_mode==false) & (quadrilater_mode==false) & (line_mode==true) ) {
          while( (!finished2d) & (point_mode==false) & (quadrilater_mode==false) ) {
            cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
            cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
            char keyboard = (char)cv::waitKey(50);
            if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false; break;}
            if(keyboard == 108) { std::cout << "'l' pressed: line mode already activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false;}
            if(keyboard == 112) { std::cout << "'p' pressed: point mode activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true; break;}
            if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; next_img=true; break;}
          }

          if ((next_img==false) & (line_mode==true)) {
            finished2d=false; finished3d=false;
            cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
            std::cout << "Point 1 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;

            // 2
            cv::setMouseCallback("Original2D", mouse_call2d, (void*)&points_2D);

            while( (!finished2d) ) {
              cv::imshow("Original2D",Image2D); cv::imshow("Original3D_without_selection",Image3D);
              cv::moveWindow("Original2D", 0, 0); cv::moveWindow("Original3D_without_selection", Image2D.cols+70, 0);
              if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl;}
            }
            finished2d=false; finished3d=false;
            cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
            std::cout << "Point 2 selected in rgb image: " << points_2D[i].x << ", " << points_2D[i].y << '\n';

            // Draw line
            std::vector<std::array<float, 2>> uv_vect; uv_vect.clear();
            sampling_line_between_two_points({points_2D[i-1].x, points_2D[i-1].y}, {points_2D[i-0].x, points_2D[i-0].y}, 8, uv_vect);
            for (size_t k = 1; k < uv_vect.size()-1; k++) {
              points_2D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) ); i=i+1;
              cv::circle(Image2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
            }
            i=i-uv_vect.size()+1;

            // 3D points
            // 1
            finished2d=false; finished3d=false;
            cv::destroyWindow("Original2D"); cv::destroyWindow("Original3D_without_selection");
            cv::namedWindow("Original3D"); cv::namedWindow("Original2D_without_selection");
            cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
            while(!finished3d) {
              cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
              cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
              if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a first point." << std::endl; }
            }
            finished2d=false; finished3d=false;
            cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
            std::cout << "Point 1 selected in depth image: " << points_3D[i].x << ", " << points_3D[i].y << '\n'; i=i+1;

            // 2
            cv::setMouseCallback("Original3D", mouse_call3d, (void*)&points_3D);
            while(!finished3d) {
              cv::imshow("Original3D",Image3D); cv::imshow("Original2D_without_selection",Image2D);
              cv::moveWindow("Original3D", 0, 0); cv::moveWindow("Original2D_without_selection", Image3D.cols+70, 0);
              if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl; }
            }
            finished2d=false; finished3d=false;
            cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
            std::cout << "Point 2 selected in depth image: " << points_3D[i].x << ", " << points_3D[i].y << '\n';

            // Draw line
            uv_vect.clear();
            sampling_line_between_two_points({points_3D[i-1].x, points_3D[i-1].y}, {points_3D[i-0].x, points_3D[i-0].y}, 8, uv_vect);
            for (size_t k = 1; k < uv_vect.size()-1; k++) {
              points_3D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) ); i=i+1;
              cv::circle(Image3D, (cvPoint(points_3D[i].x, points_3D[i].y)), 3, (255, 0, 0), -1);
            }
            i=i-uv_vect.size()+1;

            finished2d=false; finished3d=false;
            cv::destroyWindow("Original3D"); cv::destroyWindow("Original2D_without_selection");

            // save
            for (size_t m = 0; m < (uv_vect.size()); m++) {
              points_2D_yaml_save.push_back( {(double)points_2D[i].x, (double)points_2D[i].y} );
              points_3D_yaml_save.push_back( {(double)points_3D[i].x, (double)points_3D[i].y} );
              points_xyz_yaml_save.push_back({x[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], y[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ], z[ (int)(points_3D[i].y)*depth_parameters[0]+(int)(points_3D[i].x) ]});

              std::cout << "===== Point " << i << " selected ====== " << std::endl;
              std::cout << "Point selected => 2D: " << points_2D_yaml_save[i].x << ", " << points_2D_yaml_save[i].y << std::endl;
              std::cout << "Point selected => 3D: " << points_3D_yaml_save[i].x << ", " << points_3D_yaml_save[i].y << std::endl;
              std::cout << "Point selected => xyz: " << points_xyz_yaml_save[i][0] << ", " << points_xyz_yaml_save[i][1] << ", " << points_xyz_yaml_save[i][2] << std::endl << std::endl;

              i=i+1;
            }
          }
        }
      }
      k=k+1;
    }
  }
}
void OpenHSML::write_yaml_file_of_point_of_calibration(std::vector<cv::Point2d> points_3D, std::vector<cv::Point2d> points_2D, std::vector<std::array<float, 3>> points_xyz, std::string calibration_namefile) {
  std::string str_u1="[ ", str_u2="[ ", str_v1="[ ", str_v2="[ ";
  std::string str_x="[ ", str_y="[ ", str_z="[ ";
  for (size_t j=0; j<points_2D.size(); j++) {
    if ( j!=(points_2D.size()-1) ) {
      str_u1 = str_u1 + std::to_string((int)points_2D[j].x) + ", ";
      str_v1 = str_v1 + std::to_string((int)points_2D[j].y) + ", ";
      str_u2 = str_u2 + std::to_string((int)points_3D[j].x) + ", ";
      str_v2 = str_v2 + std::to_string((int)points_3D[j].y) + ", ";
      str_x = str_x + std::to_string(points_xyz[j][0]) + ", ";
      str_y = str_y + std::to_string(points_xyz[j][1]) + ", ";
      str_z = str_z + std::to_string(points_xyz[j][2]) + ", ";
    }
    else {
      str_u1 = str_u1 + std::to_string((int)points_2D[j].x) + " ]";
      str_v1 = str_v1 + std::to_string((int)points_2D[j].y) + " ]";
      str_u2 = str_u2 + std::to_string((int)points_3D[j].x) + " ]";
      str_v2 = str_v2 + std::to_string((int)points_3D[j].y) + " ]";
      str_x = str_x + std::to_string(points_xyz[j][0]) + " ]";
      str_y = str_y + std::to_string(points_xyz[j][1]) + " ]";
      str_z = str_z + std::to_string(points_xyz[j][2]) + " ]";
    }
  }

  YAML::Node node_points;
  node_points["u_2D"] = YAML::Load(str_u1); node_points["v_2D"] = YAML::Load(str_v1);
  node_points["u_3D"] = YAML::Load(str_u2); node_points["v_3D"] = YAML::Load(str_v2);
  node_points["x_3D"] = YAML::Load(str_x); node_points["y_3D"] = YAML::Load(str_y); node_points["z_3D"] = YAML::Load(str_z);

  std::ofstream outYaml;
  outYaml.open(calibration_namefile);
  outYaml << node_points;
  outYaml.close();
}
Eigen::MatrixXd OpenHSML::determine_projectionMatrix(std::vector<cv::Point2d> points_2D, std::vector<std::array<float, 3>> points_xyz) {
  Eigen::MatrixXd A(2*points_2D.size(), 11);
  Eigen::MatrixXd B(1, 2*points_2D.size());
  for (size_t j = 0; j<points_2D.size(); j++) {
    Eigen::MatrixXd point_xyz_1(4,1), point_xyz_2(4,1);
    point_xyz_2 << points_xyz[j][0], points_xyz[j][1], points_xyz[j][2], 1;
    // point_xyz_1 = Rt_2in1 * point_xyz_2;
    point_xyz_1 = point_xyz_2;
    A(0+(j*2), 0)  = point_xyz_1(0);
    A(0+(j*2), 1)  = point_xyz_1(1);
    A(0+(j*2), 2)  = point_xyz_1(2);
    A(0+(j*2), 3)  = 1;
    A(0+(j*2), 4)  = 0;
    A(0+(j*2), 5)  = 0;
    A(0+(j*2), 6)  = 0;
    A(0+(j*2), 7)  = 0;
    A(0+(j*2), 8)  = -points_2D[j].x*point_xyz_1(0);
    A(0+(j*2), 9)  = -points_2D[j].x*point_xyz_1(1);
    A(0+(j*2), 10) = -points_2D[j].x*point_xyz_1(2);

    A(1+(j*2), 0)  = 0;
    A(1+(j*2), 1)  = 0;
    A(1+(j*2), 2)  = 0;
    A(1+(j*2), 3)  = 0;
    A(1+(j*2), 4)  = point_xyz_1(0);
    A(1+(j*2), 5)  = point_xyz_1(1);
    A(1+(j*2), 6)  = point_xyz_1(2);
    A(1+(j*2), 7)  = 1;
    A(1+(j*2), 8)  = -points_2D[j].y*point_xyz_1(0);
    A(1+(j*2), 9)  = -points_2D[j].y*point_xyz_1(1);
    A(1+(j*2), 10) = -points_2D[j].y*point_xyz_1(2);

    B(0, 0+(j*2))  = points_2D[j].x;
    B(0, 1+(j*2))  = points_2D[j].y;
  }
  Eigen::MatrixXd X(11, 1), P(3, 4);
  Eigen::MatrixXd pinv = A.completeOrthogonalDecomposition().pseudoInverse();
  X = pinv*B.transpose() ; // X = A.pseudoInverse()*B.transpose() ;
  P <<  X(0), X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10), 1;
  return (P); // return (P*Rt_2in1);
}
void OpenHSML::determine_fundamentalMatrix_and_projectionMatrix(cv::Mat& fundamental_matrix, Eigen::MatrixXd& projection_matrix, std::string calibration_namefile) {
  std::vector<cv::Point2d> points_2D, points_3D; points_2D.clear(), points_3D.clear();
  std::vector<std::array<float, 3>> points_xyz;points_xyz.clear();
  read_yaml_file_of_point_of_calibration(calibration_namefile+"/points_of_calibration.yaml", points_3D, points_2D, points_xyz);

  fundamental_matrix = cv::findFundamentalMat(points_3D, points_2D, CV_FM_RANSAC); // other options: CV_FM_7POINT, CV_FM_8POINT, CV_FM_RANSAC, CV_FM_LMEDS
  projection_matrix = determine_projectionMatrix(points_2D, points_xyz);
}
void OpenHSML::check_parameters_of_calibration_and_change_if_necessary(std::string calibration_namefile) {
  std::vector<cv::Point2d> points_2D, points_3D;
  std::vector<std::array<float, 3>> points_xyz;
  std::array<float, 3> alpha_beta_gamma, xyz_translation;
  points_2D.clear(), points_3D.clear(); points_xyz.clear();

  std::vector<double> depth_parameters;
  read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);
  modification_of_camera_parameters(depth_parameters);
  write_yaml_file_of_calibration(depth_parameters, calibration_namefile+"/calibration.yaml");

  read_yaml_file_of_point_of_calibration(calibration_namefile+"/points_of_calibration.yaml", points_3D, points_2D, points_xyz);
  modification_of_calibration_of_points(points_3D, points_2D, points_xyz, depth_parameters, calibration_namefile);
  write_yaml_file_of_point_of_calibration(points_3D, points_2D, points_xyz, calibration_namefile+"/points_of_calibration.yaml");

  determine_fundamentalMatrix_and_projectionMatrix(fundamental_matrix, projection_matrix, calibration_namefile);
  write_yaml_file_of_calibration(depth_parameters, calibration_namefile+"/calibration.yaml");
}

void OpenHSML::save_image_with_points_and_epipolar_lines(std::string calibration_namefile, std::string nameFile_toSave) {
  // Read yaml file
  std::vector<cv::Point2d> points_2D, points_3D; points_2D.clear(), points_3D.clear();
  std::vector<std::array<float, 3>> points_xyz;points_xyz.clear();
  std::vector<double> depth_parameters;

  read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);
  read_yaml_file_of_point_of_calibration(calibration_namefile+"/points_of_calibration.yaml", points_3D, points_2D, points_xyz);

  cv::Mat Image2DWithPoints(cvSize(depth_parameters[0], depth_parameters[1]), CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat Image3DWithEpipolarLines(cvSize(depth_parameters[0], depth_parameters[1]), CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat Image2DWithEpipolarLines(cvSize(depth_parameters[0], depth_parameters[1]), CV_8UC3, cv::Scalar(255, 255, 255));

  // Determine epipolar lines equation
  std::vector<cv::Vec3d> leftLines, rightLines;
  cv::computeCorrespondEpilines(points_3D, 1, fundamental_matrix.t(), rightLines);
  cv::computeCorrespondEpilines(points_2D, 2, fundamental_matrix, leftLines);

  for(std::size_t i=0; i<leftLines.size(); i=i+1) {
    // To draw the line on the image - resolve: ax+by+c=0
    cv::Vec3d l_right=rightLines.at(i);
    double x0_opencv_right,y0_opencv_right,x1_opencv_right,y1_opencv_right, a_opencv_right=l_right.val[0], b_opencv_right=l_right.val[1], c_opencv_right=l_right.val[2];
    x0_opencv_right=0; y0_opencv_right=(-c_opencv_right-a_opencv_right*x0_opencv_right)/b_opencv_right; x1_opencv_right=Image2DWithEpipolarLines.cols; y1_opencv_right=(-c_opencv_right-a_opencv_right*x1_opencv_right)/b_opencv_right;

    cv::Vec3d l_left=leftLines.at(i);
    double x0_opencv_left,y0_opencv_left,x1_opencv_left,y1_opencv_left, a_opencv_left=l_left.val[0], b_opencv_left=l_left.val[1], c_opencv_left=l_left.val[2];
    x0_opencv_left=0; y0_opencv_left=(-c_opencv_left-a_opencv_left*x0_opencv_left)/b_opencv_left; x1_opencv_left=Image3DWithEpipolarLines.cols; y1_opencv_left=(-c_opencv_left-a_opencv_left*x1_opencv_left)/b_opencv_left;

    cv::line(Image2DWithEpipolarLines, cvPoint(x0_opencv_right,y0_opencv_right), cvPoint(x1_opencv_right,y1_opencv_right), cv::Scalar(0, 0, 0), 1);
    cv::line(Image3DWithEpipolarLines, cvPoint(x0_opencv_left,y0_opencv_left), cvPoint(x1_opencv_left,y1_opencv_left), cv::Scalar(0, 0, 0), 1);
    cv::circle(Image2DWithPoints, (cv::Point2d(points_2D.at(i).x, points_2D.at(i).y)), 2, cv::Scalar(0, 0, 0), -1);
  }

  cv::imwrite(nameFile_toSave+"/Image2DWithEpipolarLines.jpg", Image2DWithEpipolarLines);
  cv::imwrite(nameFile_toSave+"/Image3DWithEpipolarLines.jpg", Image3DWithEpipolarLines);
  cv::imwrite(nameFile_toSave+"/Image2DWithPoints.jpg", Image2DWithPoints);
}

// Global function:
void OpenHSML::select_point_manually_in_image(cv::Mat Image2DRGB, std::vector<float>& u, std::vector<float>& v) {
  std::vector<cv::Point> points_2D;
  cv::Mat img_2D = Image2DRGB.clone();
  int i=0;
  bool end_selection=false, quadrilater_mode=false, line_mode=false, point_mode=true;
  while ( end_selection==false ) { // signal echap pour passer aux images suivante et tant qu'il y en a
    cv::namedWindow("select point(s) in this image");
    cv::setMouseCallback("select point(s) in this image", mouse_call2d, (void*)&points_2D);

    // mode normal:
    if ( (point_mode==true) & (quadrilater_mode==false) & (line_mode==false) ) {
      while( (!finished2d) & (quadrilater_mode==false) & (line_mode==false) ) {
        cv::imshow("select point(s) in this image",img_2D);
        char keyboard = (char)cv::waitKey(50);
        if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false; break;}
        if(keyboard == 108) { std::cout << "'l' pressed: line mode activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false; break;}
        if(keyboard == 112) { std::cout << "'p' pressed: point mode already activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true;}
        if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; end_selection=true; break;}
      }
      if ((end_selection==false) & (point_mode==true)) {
        finished2d=false;
        cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
        std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n';
        i=i+1;
      }
    }

    // mode quadrilater:
    if ( (point_mode==false) & (quadrilater_mode==true) & (line_mode==false) ) {
      while( (!finished2d) & (point_mode==false) & (line_mode==false) ) {
        cv::imshow("select point(s) in this image",img_2D);
        char keyboard = (char)cv::waitKey(50);
        if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode already activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false;}
        if(keyboard == 108) { std::cout << "'l' pressed: line mode activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false; break;}
        if(keyboard == 112) { std::cout << "'p' pressed: point mode activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true; break;}
        if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; end_selection=true; break;}
      }
      if ((end_selection==false) & (quadrilater_mode==true)) {
        finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
        std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        while( (!finished2d) ) {
          cv::imshow("select point(s) in this image",img_2D);
          if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl;}
        }
        if (end_selection==false) {
          finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        }
        while( (!finished2d) ) {
          cv::imshow("select point(s) in this image",img_2D);
          if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a third point." << std::endl;}
        }
        if (end_selection==false) {
          finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        }
        while( (!finished2d) ) {
          cv::imshow("select point(s) in this image",img_2D);
          if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a fourth point." << std::endl;}
        }
        if (end_selection==false) {
          finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        }
        // ajouter tout les points avec la fonction
        std::vector<std::array<float, 2>> uv_vect; uv_vect.clear();
        sampling_square_with_four_points( {points_2D[i-4].x, points_2D[i-4].y},
          {points_2D[i-3].x, points_2D[i-3].y},
          {points_2D[i-2].x, points_2D[i-2].y},
          {points_2D[i-1].x, points_2D[i-1].y},
          8, uv_vect); // ????
          for (size_t k = 0; k < uv_vect.size(); k++) {
            points_2D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) );
            cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1); i=i+1;
          }
      }
    }

    // mode ligne:
    if ( (point_mode==false) & (quadrilater_mode==false) & (line_mode==true) ) {
      while( (!finished2d) & (point_mode==false) & (quadrilater_mode==false) ) {
        cv::imshow("select point(s) in this image",img_2D);
        char keyboard = (char)cv::waitKey(50);
        if(keyboard == 113) { std::cout << "'q' pressed: quadrilater mode activated" << std::endl; quadrilater_mode=true; line_mode=false; point_mode=false; break;}
        if(keyboard == 108) { std::cout << "'l' pressed: line mode already activated" << std::endl; quadrilater_mode=false; line_mode=true; point_mode=false;}
        if(keyboard == 112) { std::cout << "'p' pressed: point mode activated" << std::endl; quadrilater_mode=false; line_mode=false; point_mode=true; break;}
        if(keyboard == 27) { std::cout << "'ESC' pressed: end of selection" << std::endl; end_selection=true; break;}
      }
      if ((end_selection==false) & (line_mode==true)) {
        finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
        std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        while( (!finished2d) ) {
          cv::imshow("select point(s) in this image",img_2D);
          if((char)cv::waitKey(50) == 27) { std::cout << "'ESC' pressed: you can't exit here. Select a second point." << std::endl;}
        }
        if (end_selection==false) {
          finished2d=false; cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1);
          std::cout << "Point selected: " << points_2D[i].x << ", " << points_2D[i].y << '\n'; i=i+1;
        }
        // ajouter tout les points avec la fonction
        std::vector<std::array<float, 2>> uv_vect; uv_vect.clear();
        sampling_line_between_two_points({points_2D[i-2].x, points_2D[i-2].y}, {points_2D[i-1].x, points_2D[i-1].y}, 8, uv_vect);
        for (size_t k = 0; k < uv_vect.size(); k++) {
          points_2D.push_back( cv::Point(uv_vect[k][0], uv_vect[k][1]) );
          cv::circle(img_2D, (cvPoint(points_2D[i].x, points_2D[i].y)), 3, (255, 0, 0), -1); i=i+1;
        }
      }
    }

  }

  cv::destroyWindow("select point(s) in this image");

  u.clear(); v.clear();
  for (size_t j=0; j<points_2D.size(); j++) {
    u.push_back( points_2D[j].x );
    v.push_back( points_2D[j].y );
  }
}
void OpenHSML::sampling_line_between_two_points(std::array<float, 2> pt_A, std::array<float, 2> pt_B, float nb_of_pt, std::vector<std::array<float, 2>>& pts) {
  if ( pt_A[0]==pt_B[0] ) {
    pt_A[0]++;
  }

  float a=(pt_B[1]-pt_A[1])/(pt_B[0]-pt_A[0]);
  float b=pt_A[1]-a*pt_A[0];

  std::array<float, 2> uv; pts.clear();
  if ( ((-M_PI/4)<=a) & (a<=(M_PI/4)) ) {
    float step, x_, xmax;
    if (pt_A[0]<=pt_B[0]) { step=(pt_B[0]-pt_A[0])/(nb_of_pt+1); x_=pt_A[0]; xmax=pt_B[0]; }
    else { step=(pt_A[0]-pt_B[0])/(nb_of_pt+1); x_=pt_B[0]; xmax=pt_A[0]; }
    while (x_<=(xmax+step/3)) {
      uv[1] = a*x_+b; uv[0] = x_;
      pts.push_back( uv );
      x_=x_+step;
    }
    if ( pt_A[0]>=pt_B[0] ) { reverse(pts.begin(), pts.end()); }
  }
  else {
    float step, y_, ymax;
    if (pt_A[1]<=pt_B[1]) { step=(pt_B[1]-pt_A[1])/(nb_of_pt+1); y_=pt_A[1]; ymax=pt_B[1]; }
    else { step=(pt_A[1]-pt_B[1])/(nb_of_pt+1); y_=pt_B[1]; ymax=pt_A[1]; }
    while (y_<=(ymax+step/3)) {
      uv[0] = (y_-b)/a; uv[1] = y_;
      pts.push_back( uv );
      y_=y_+step;
    }
    if ( pt_A[1]>=pt_B[1] ) { reverse(pts.begin(), pts.end()); }
  }
}
void OpenHSML::sampling_square_with_four_points(std::array<float, 2> pt_A, std::array<float, 2> pt_B, std::array<float, 2> pt_C, std::array<float, 2> pt_D, float nb_of_pt_by_line, std::vector<std::array<float, 2>>& pts) {
  std::vector<std::array<float, 2>> uv_vect_btw_1_2; uv_vect_btw_1_2.clear();
  sampling_line_between_two_points({pt_A[0], pt_A[1]}, {pt_B[0], pt_B[1]}, nb_of_pt_by_line, uv_vect_btw_1_2);
  std::vector<std::array<float, 2>> uv_vect_btw_4_3; uv_vect_btw_4_3.clear();
  sampling_line_between_two_points({pt_D[0], pt_D[1]}, {pt_C[0], pt_C[1]}, nb_of_pt_by_line, uv_vect_btw_4_3);

  // save:
  for (size_t k=1; k<(uv_vect_btw_1_2.size()-1); k++) { pts.push_back( {uv_vect_btw_1_2[k][0], uv_vect_btw_1_2[k][1]} ); }
  for (size_t k=1; k<(uv_vect_btw_4_3.size()-1); k++) { pts.push_back( {uv_vect_btw_4_3[k][0], uv_vect_btw_4_3[k][1]} ); }

  std::vector<std::array<float, 2>> uv_vect_btw_l12_l43;
  for (size_t m = 0; m < uv_vect_btw_1_2.size(); m++) {
    uv_vect_btw_l12_l43.clear();
    sampling_line_between_two_points( {uv_vect_btw_1_2[m][0], uv_vect_btw_1_2[m][1]}, {uv_vect_btw_4_3[m][0], uv_vect_btw_4_3[m][1]}, nb_of_pt_by_line, uv_vect_btw_l12_l43);

    for (size_t k=1; k<(uv_vect_btw_l12_l43.size()-1); k++) { pts.push_back( {uv_vect_btw_l12_l43[k][0], uv_vect_btw_l12_l43[k][1]} ); }
  }
}

void OpenHSML::sampling_epipolar_lines(double a, double b, double c, float nb_of_pt, std::vector<std::array<float, 2>>& uv_vect, int imageWidth, int imageHeight) {
  // Recover the different points of the sampled line in pixel value (on 3D image)
  float xmin, xmax, ymin, ymax;
  if ( (0<=((-c-a*0)/b)) & (((-c-a*0)/b)<=imageHeight) ) { xmin=0; ymin=((-c-a*xmin)/b); }
  else if ( (0<=((-c-b*0)/a)) & (((-c-b*0)/a)<=imageWidth) ) { ymin=0; xmin=((-c-b*ymin)/a); }
  else { ymin=imageHeight; xmin=((-c-b*ymin)/a); }

  if ( (0<=((-c-a*imageWidth)/b)) & (((-c-a*imageWidth)/b)<=imageHeight) ) { xmax=imageWidth; ymax=((-c-a*xmax)/b); }
  else if ( (0<=((-c-b*imageHeight)/a)) & (((-c-b*imageHeight)/a)<=imageWidth) ) { ymax=imageHeight; xmax=((-c-b*ymax)/a); }
  else { ymax=0; xmax=((-c-b*ymax)/a); }

  sampling_line_between_two_points({xmin, ymin}, {xmax, ymax}, nb_of_pt, uv_vect);
}
void OpenHSML::projection_of_sampled_points_in_2D_image(Eigen::MatrixXd projection_matrix, std::vector<std::array<float, 3>> xyz_vect, std::vector<std::array<float, 3>>& m_2D_vect) {
  std::array<float, 3> m_2D; m_2D_vect.clear();
  Eigen::MatrixXd m(3,1); Eigen::MatrixXd M_3D(4,1);
  for (size_t j=0; j<xyz_vect.size(); j++) {
    M_3D << xyz_vect[j][0], xyz_vect[j][1], xyz_vect[j][2], 1;
    m = projection_matrix * M_3D;
    m_2D[0] = m(0,0)/m(2,0); m_2D[1] = m(1,0)/m(2,0); m_2D[2] = m(2,0)/m(2,0);
    m_2D_vect.push_back( m_2D );
  }
}
void OpenHSML::estimation_of_the_points_closest_to_the_initial_points(std::vector<std::array<float, 3>> xyz_vect, std::vector<std::array<float, 2>> uv_vect, std::vector<std::array<float, 3>> m_2D_vect, float input_u, float input_v, float& output_u, float& output_v, float& output_x, float& output_y, float& output_z) {
  std::vector<float> distance; distance.clear();
  for (size_t j=0; j<m_2D_vect.size(); j++) {
    if ( ((xyz_vect[j][2] <= -0.00001)|(0.00001 <= xyz_vect[j][2])) ) {
      distance.push_back( sqrt( std::pow(m_2D_vect[j][0]-input_u, 2) + std::pow(m_2D_vect[j][1]-input_v, 2) ) );
    }
    else { distance.push_back( 1000 ); }
  }

  const auto [min, max] = std::minmax_element(begin(distance), end(distance));
  for (size_t j=0; j<distance.size(); j++) {
    if ( distance[j] == *min ) {
      // save 3D points estimated in pixels
      output_u=uv_vect[j][0];
      output_v=uv_vect[j][1];

      // save 3D points estimated in voxels
      output_x=xyz_vect[j][0];
      output_y=xyz_vect[j][1];
      output_z=xyz_vect[j][2];
    }
  }
}
void OpenHSML::display_or_save_all_img(std::vector<float> input_u, std::vector<float> input_v, std::vector<float> output_u, std::vector<float> output_v, std::vector<std::vector<std::array<float, 2>>> uv_vvect, std::vector<std::vector<std::array<float, 3>>> m_2D_vvect, cv::Mat input_img_2D, cv::Mat input_img_3D, bool arg_display, bool arg_save, std::string nameFile_toSave) {
  // Create image
  cv::Mat img_2D = input_img_2D.clone();
  cv::Mat img_3D = input_img_3D.clone();
  cv::Mat img2D_pt_desired = input_img_2D.clone();
  cv::Mat img3D_epipolar_lines = input_img_3D.clone();
  cv::Mat img3D_epipolar_lines_sampling = input_img_3D.clone();
  cv::Mat img2D_pt_estimated_sampling_points = input_img_2D.clone();
  cv::Mat img3D_pt_estimated = input_img_3D.clone();

  // std::vector<uint8_t> myArray_data_img(input_img_2D.cols*input_img_2D.rows*3);
  std::vector<uint8_t> myVec;
  cv::Mat input_img_2D_uint8; input_img_2D.convertTo(input_img_2D_uint8, CV_8UC3);
  myVec.assign((uint8_t*)input_img_2D_uint8.datastart, (uint8_t*)input_img_2D_uint8.dataend);

  // Open file: Python
  std::ofstream file_to_display_with_python;
  file_to_display_with_python.open( "/tmp/pts.txt" );

  input_points.r.clear(); input_points.g.clear(); input_points.b.clear();
  for(std::size_t i=0; i<input_points.u.size(); i=i+1) {
    if ( (input_points.u[i]!=0) && (input_points.v[i]!=0) ) {
      // Create color
      cv::Scalar color;
      input_points.r.push_back( myVec[((int)(input_points.v[i])*input_img_2D.cols+(int)(input_points.u[i]))*3+0] );
      input_points.g.push_back( myVec[((int)(input_points.v[i])*input_img_2D.cols+(int)(input_points.u[i]))*3+1] );
      input_points.b.push_back( myVec[((int)(input_points.v[i])*input_img_2D.cols+(int)(input_points.u[i]))*3+2] );
      color = cvScalar(input_points.r[i], input_points.g[i], input_points.b[i]);

      // Draw
      cv::circle(img2D_pt_desired, (cvPoint(input_u[i],input_v[i])),3, color, -1);
      cv::line(img3D_epipolar_lines,cvPoint(uv_vvect[i][0][0],uv_vvect[i][0][1]),cvPoint(uv_vvect[i][uv_vvect[i].size()-1][0],uv_vvect[i][uv_vvect[i].size()-1][1]),color, 1);
      for (size_t j=0; j<uv_vvect[i].size(); j++) { cv::circle(img3D_epipolar_lines_sampling, (cvPoint(uv_vvect[i][j][0],uv_vvect[i][j][1])),3, color, -1); }
      for (size_t j=0; j<m_2D_vvect[i].size(); j++) { cv::circle(img2D_pt_estimated_sampling_points, (cvPoint(m_2D_vvect[i][j][0],m_2D_vvect[i][j][1])),3, color, -1); }
      cv::circle(img3D_pt_estimated   , (cvPoint(output_u[i],output_v[i])),3, color, -1);

      // Python
      file_to_display_with_python << output_points.x[i] << " " << output_points.y[i] << " " << output_points.z[i] << " " << input_points.r[i]  << " " << input_points.g[i]  << " " << input_points.b[i]  << std::endl;
    }
  }
  // Python
  file_to_display_with_python.close();

  // Save
  if ( arg_save==true ) {
    cv::imwrite(nameFile_toSave+"a.jpg", img_2D);
    cv::imwrite(nameFile_toSave+"b.jpg", img_3D);
    cv::imwrite(nameFile_toSave+"c.jpg", img2D_pt_desired);
    cv::imwrite(nameFile_toSave+"d.jpg", img3D_epipolar_lines);
    cv::imwrite(nameFile_toSave+"e.jpg", img3D_epipolar_lines_sampling);
    cv::imwrite(nameFile_toSave+"f.jpg", img2D_pt_estimated_sampling_points);
    cv::imwrite(nameFile_toSave+"g.jpg", img3D_pt_estimated);
  }

  // Display
  if ( arg_display==true ) {
    cv::namedWindow("the start: Image_2DRGB", CV_8UC3); cv::imshow("the start: Image_2DRGB", img_2D);
    cv::namedWindow("the start: Image_3DRGB", CV_8UC3); cv::imshow("the start: Image_3DRGB", img_3D);
    cv::waitKey();
    cv::namedWindow("the explication: Image_2DRGB_WithDesiredPoints", CV_8UC3); cv::imshow("the explication: Image_2DRGB_WithDesiredPoints", img2D_pt_desired);
    cv::waitKey();
    cv::namedWindow("the explication: Image_3DRGB_WithEpipolarLines", CV_8UC3); cv::imshow("the explication: Image_3DRGB_WithEpipolarLines", img3D_epipolar_lines);
    cv::waitKey();
    cv::namedWindow("the explication: Image_3DRGB_WithEpipolarLinesSampling", CV_8UC3); cv::imshow("the explication: Image_3DRGB_WithEpipolarLinesSampling", img3D_epipolar_lines_sampling);
    cv::waitKey();
    cv::namedWindow("the explication: Image_2DRGB_WithEstimationOfSamplingPoints", CV_8UC3); cv::imshow("the explication: Image_2DRGB_WithEstimationOfSamplingPoints", img2D_pt_estimated_sampling_points);
    cv::waitKey();
    cv::namedWindow("the end: Image_3DRGB_EstimatedPoints", CV_8UC3); cv::imshow("the end: Image_3DRGB_EstimatedPoints", img3D_pt_estimated);
    cv::waitKey();

    cv::destroyWindow("the start: Image_2DRGB");
    cv::destroyWindow("the start: Image_3DRGB");
    cv::destroyWindow("the explication: Image_2DRGB_WithDesiredPoints");
    cv::destroyWindow("the explication: Image_3DRGB_WithEpipolarLines");
    cv::destroyWindow("the explication: Image_3DRGB_WithEpipolarLinesSampling");
    cv::destroyWindow("the explication: Image_2DRGB_WithEstimationOfSamplingPoints");
    cv::destroyWindow("the end: Image_3DRGB_EstimatedPoints");

    // // matplotlib-cpp:
    // matplotlibcpp::plot3(output_points.x, output_points.y, output_points.z, {{"color", "red"}, {"marker", "o"}, {"ls", ""}});
  	// matplotlibcpp::xlabel("x"); matplotlibcpp::ylabel("y"); matplotlibcpp::set_zlabel("z");
  	// matplotlibcpp::legend();
  	// matplotlibcpp::show();

    // // matplotlib-python:
    // std::string folder_name = "src/display_pts.py";
    // std::string comand_system="python "+folder_name;
    // std::system(comand_system.c_str());
  }
}
void OpenHSML::stereovision_hybrid(cv::Mat fundamental_matrix, Eigen::MatrixXd projection_matrix, struct Input_Points input_points, struct Input_Images input_img, struct Output_Points& output_points, bool arg_display, bool arg_save, std::string calibration_namefile, std::string nameFile_toSave) {
  std::vector<cv::Point2d> Pts_2D;
  for (size_t i=0; i<input_points.u.size(); i++) {
    Pts_2D.push_back(cv::Point2d(input_points.u[i], input_points.v[i]));
  }
  std::vector<cv::Vec3d> EquationLines_3D;
  cv::computeCorrespondEpilines(Pts_2D, 2, fundamental_matrix, EquationLines_3D);
  // Initialise output
  output_points.u.clear(); output_points.v.clear(); output_points.x.clear(); output_points.y.clear();  output_points.z.clear();
  for (size_t j=0; j<input_points.u.size()+1; j++) {
    output_points.u.push_back(0.0); output_points.v.push_back(0.0); output_points.x.push_back(0.0); output_points.y.push_back(0.0); output_points.z.push_back(0.0);
  }

  std::vector<double> depth_parameters;
  read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);
  std::vector<std::vector<std::array<float, 2>>> uv_vvect;
  std::vector<std::vector<std::array<float, 3>>> m_2D_vvect;
  for(std::size_t i=0; i<EquationLines_3D.size(); i=i+1) {
    if ( (input_points.u[i]!=0) && (input_points.v[i]!=0) ) {
      cv::Vec3d l_3D=EquationLines_3D.at(i);
      double a=l_3D.val[0], b=l_3D.val[1], c=l_3D.val[2];

      std::array<float, 3> xyz;
      std::vector<std::array<float, 2>> uv_vect_;
      sampling_epipolar_lines(a, b, c, 75, uv_vect_, depth_parameters[0], depth_parameters[1]);
      std::vector<std::array<float, 3>> xyz_vect; xyz_vect.clear();

      std::vector<std::array<float, 2>> uv_vect;

      for (size_t j = 0; j < uv_vect_.size(); j++) {
        xyz = { input_img.pt_x[ (int)(uv_vect_[j][1])*depth_parameters[0]+(int)(uv_vect_[j][0]) ],
                input_img.pt_y[ (int)(uv_vect_[j][1])*depth_parameters[0]+(int)(uv_vect_[j][0]) ],
                input_img.pt_z[ (int)(uv_vect_[j][1])*depth_parameters[0]+(int)(uv_vect_[j][0]) ] };
        xyz_vect.push_back(xyz);
        uv_vect.push_back(uv_vect_[j]);
      }

      std::vector<std::array<float, 3>> m_2D_vect;
      projection_of_sampled_points_in_2D_image(projection_matrix, xyz_vect, m_2D_vect);
      estimation_of_the_points_closest_to_the_initial_points(xyz_vect, uv_vect, m_2D_vect, input_points.u[i], input_points.v[i], output_points.u[i], output_points.v[i], output_points.x[i], output_points.y[i], output_points.z[i]);

      if ( (arg_display==true) | (arg_save==true) ) {
        uv_vvect.push_back(uv_vect); m_2D_vvect.push_back(m_2D_vect);
      }
    }
  }

  if ( (arg_display==true) | (arg_save==true) ) {
    display_or_save_all_img(input_points.u, input_points.v, output_points.u, output_points.v, uv_vvect, m_2D_vvect, input_img.Image2DRGB, input_img.Img3DGS, arg_display, arg_save, nameFile_toSave);
  }
}

#endif
