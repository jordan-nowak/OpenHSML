/**
	* @file demo.cpp
  * @brief
  *
	* @author Jordan Nowak
	* @version 1.0.0
	* @date 31-10-2020
	*/
#include <OpenHSML.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <experimental/filesystem>

#include <iostream>

int main(int argc, char* argv[]) {
  std::cout<<"\x1B[1m"<<"-- Start openhsml program"<<"\x1B[0m"<<std::endl;

  OpenHSML stv;

  std::string nameFile_toSave, Image2D_path, Image3D_path, ImageXYZ_path;
  bool arg_calibration=false, arg_display=false, arg_test=false, arg_img=false, arg_save=false;
  if (argc<2) {
    std::cerr << std::endl << std::endl;
    std::cerr << "\x1B[1;31m" << "************************************************************************" << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << " - To realize the calibration   '-calibration'                          " << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << " - To realize the test          '-test'                                 " << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << " - To realize the display       '-display'                              " << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << " - To realize the save-         '-save path/to/save'                    " << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << " - To give image path           '-img img_2D img_3D file_XYZ'           " << "\x1B[0m" << std::endl;
    std::cerr << "\x1B[1;31m" << "************************************************************************" << "\x1B[0m" << std::endl;
    exit(0);
  }
  else {
  	for (int i=1; i<argc; i++) {
  		std::string arg_ = argv[i];
  		if ( arg_=="-calibration"  ) { arg_calibration=true; }
  		if ( arg_=="-display"  ) { arg_display=true; }
      if ( arg_=="-test"  ) { arg_test=true; }
      if ( arg_=="-img"  ) { arg_img=true; Image2D_path=argv[i+1]; ImageXYZ_path=argv[i+2]; }
  		if ( arg_=="-save"  ) { arg_save=true; nameFile_toSave=argv[i+1]; }
  	}
  }

  std::string wksp = getenv("wksp");
  std::string calibration_namefile = wksp + "/packages/openhsml/share/resources/calibration";

  if ( arg_calibration==true ) {
      stv.check_parameters_of_calibration_and_change_if_necessary(calibration_namefile);
  }
  
  if (arg_save==true) {
    stv.save_image_with_points_and_epipolar_lines(calibration_namefile, nameFile_toSave);
  }

  if ( (arg_test==true) & (arg_img==false)) {
    std::vector<std::string> name_file_2d;
    for(auto& p: std::experimental::filesystem::directory_iterator(calibration_namefile+"/../img/2d"))
    name_file_2d.push_back(p.path());
    std::sort(name_file_2d.begin(), name_file_2d.end());

    std::vector<std::string> name_file_3d;
    for(auto& p: std::experimental::filesystem::directory_iterator(calibration_namefile+"/../img/depth"))
    name_file_3d.push_back(p.path());
    std::sort(name_file_3d.begin(), name_file_3d.end());

    std::vector<double> vect;
    stv.input_img.ImageXYZ_path = name_file_3d[0];
    stv.input_img.Image2DRGB = cv::imread(name_file_2d[0]);
    stv.convert_yaml_file_to_vector(stv.input_img.ImageXYZ_path, vect);
    stv.convert_vector_to_xyz_vector(vect, calibration_namefile, stv.input_img.pt_x, stv.input_img.pt_y, stv.input_img.pt_z);

    std::vector<double> depth_parameters;
    stv.read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);
    stv.convert_depth_vector_to_img_grayscale(stv.input_img.pt_z, stv.input_img.Img3DGS, depth_parameters[0], depth_parameters[1]);
  }

  // Retrieve image
  if ( arg_img==true ) {
    stv.input_img.Image2DRGB = cv::imread(Image2D_path);

    std::vector<double> depth_parameters;
    stv.read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);

    std::vector<double> vect;
    stv.input_img.ImageXYZ_path = ImageXYZ_path;
    stv.convert_yaml_file_to_vector(stv.input_img.ImageXYZ_path, vect);
    stv.convert_vector_to_xyz_vector(vect, calibration_namefile, stv.input_img.pt_x, stv.input_img.pt_y, stv.input_img.pt_z);
    stv.convert_depth_vector_to_img_grayscale(stv.input_img.pt_z, stv.input_img.Img3DGS, depth_parameters[0], depth_parameters[1]);
  }

  // Select point manually
  if ((arg_img==true) | (arg_test==true)) {
    stv.select_point_manually_in_image(stv.input_img.Image2DRGB, stv.input_points.u, stv.input_points.v);

    // To do stereovision between an image 3D and 2D
    std::vector<double> depth_parameters; cv::Mat output_img;
    stv.read_yaml_file_of_calibration(calibration_namefile+"/calibration.yaml", depth_parameters);
    stv.stereovision_hybrid(stv.fundamental_matrix, stv.projection_matrix, stv.input_points, stv.input_img, stv.output_points, output_img, arg_display, arg_save, calibration_namefile, nameFile_toSave);
  }

  std::cout<<"\x1B[1m"<<"-- End openhsml program"<<"\x1B[0m"<<std::endl;
	return 0;
}
