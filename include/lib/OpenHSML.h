/**
	* @file OpenHSML.h
  * @brief
  *
	* @author Jordan Nowak
	* @version 1.0.0
	* @date 31-10-2020
	*/
#include "Eigen"
#include <yaml-cpp/yaml.h>
#include <experimental/filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#ifndef OPENHSML_H
#define OPENHSML_H

static void mouse_call(int event,int x,int y, int, void* param);
static void mouse_call2d(int event,int x,int y, int, void* param);
static void mouse_call3d(int event,int x,int y, int, void* param);
std::vector<std::string> split(const std::string& s, char delimiter);

/**
  * @brief
  */
class OpenHSML {
  public :
      OpenHSML();
      ~OpenHSML();

      // Variables
      Eigen::MatrixXd projection_matrix;
      cv::Mat fundamental_matrix;

      // Structures
      struct Input_Images {
        cv::Mat Image2DRGB, Img3DGS;
        std::string ImageXYZ_path;
        std::vector<float> pt_x, pt_y, pt_z;
      };
      struct Input_Points { std::vector<float> u, v; std::vector<int> r, g, b; };
      struct Output_Points { std::vector<float> u, v, x, y, z; };

      struct Input_Images input_img;
      struct Input_Points input_points;
      struct Output_Points output_points;

      // Reading:
      void convert_yaml_file_to_cvmat(std::string yml_img_namefile, cv::Mat& img);
      void convert_cvmat_to_vector(cv::Mat img, std::vector<double>& myVec);
      void convert_yaml_file_to_vector(std::string yml_img_namefile, std::vector<double>& myVec);
      void convert_vector_to_xyz_vector(std::vector<double> vect, std::string calibration_namefile, std::vector<float>& x_vect, std::vector<float>& y_vect, std::vector<float>& z_vect);
      void convert_depth_vector_to_img_grayscale(std::vector<float> z_vect, cv::Mat& ImgGrayScale, int imageWidth, int imageHeight);

      // Calibration:
      void read_yaml_file_of_calibration(std::string calibration_namefile, std::vector<double>& depth_parameters);
      void read_yaml_file_of_point_of_calibration(std::string calibration_namefile, std::vector<cv::Point2d>& points_3D, std::vector<cv::Point2d>& points_2D, std::vector<std::array<float, 3>>& points_xyz);
      void modification_of_camera_parameters(std::vector<double>& depth_parameters);
      void write_yaml_file_of_calibration(std::vector<double> depth_parameters, std::string calibration_namefile);
      void modification_of_calibration_of_points(std::vector<cv::Point2d>& points_3D_yaml_save, std::vector<cv::Point2d>& points_2D_yaml_save, std::vector<std::array<float, 3>>& points_xyz_yaml_save, std::vector<double>& depth_parameters, std::string calibration_namefile);
      void write_yaml_file_of_point_of_calibration(std::vector<cv::Point2d> points_3D, std::vector<cv::Point2d> points_2D, std::vector<std::array<float, 3>> points_xyz, std::string calibration_namefile);
      Eigen::MatrixXd determine_projectionMatrix(std::vector<cv::Point2d> points_2D, std::vector<std::array<float, 3>> points_xyz);
      void determine_fundamentalMatrix_and_projectionMatrix(cv::Mat& fundamental_matrix, Eigen::MatrixXd& projection_matrix, std::string calibration_namefile);
      void check_parameters_of_calibration_and_change_if_necessary(std::string calibration_namefile);
      void save_image_with_points_and_epipolar_lines(std::string calibration_namefile, std::string nameFile_toSave="/tmp");

      // Global function:
      void select_point_manually_in_image(cv::Mat Image2DRGB, std::vector<float>& u, std::vector<float>& v);
      void sampling_line_between_two_points(std::array<float, 2> pt_A, std::array<float, 2> pt_B, float nb_of_pt, std::vector<std::array<float, 2>>& pts);
      void sampling_square_with_four_points(std::array<float, 2> pt_A, std::array<float, 2> pt_B, std::array<float, 2> pt_C, std::array<float, 2> pt_D, float nb_of_pt_by_line, std::vector<std::array<float, 2>>& pts);
      void sampling_epipolar_lines(double a, double b, double c, float nb_of_pt, std::vector<std::array<float, 2>>& uv_vect, int imageWidth, int imageHeight);
      void projection_of_sampled_points_in_2D_image(Eigen::MatrixXd projection_matrix, std::vector<std::array<float, 3>> xyz_vect, std::vector<std::array<float, 3>>& m_2D_vect);
      void estimation_of_the_points_closest_to_the_initial_points(std::vector<std::array<float, 3>> xyz_vect, std::vector<std::array<float, 2>> uv_vect, std::vector<std::array<float, 3>> m_2D_vect, float input_u, float input_v, float& output_u, float& output_v, float& output_x, float& output_y, float& output_z);
      void display_or_save_all_img(std::vector<float> input_u, std::vector<float> input_v, std::vector<float> output_u, std::vector<float> output_v, std::vector<std::vector<std::array<float, 2>>> uv_vvect, std::vector<std::vector<std::array<float, 3>>> m_2D_vvect, cv::Mat input_img_2D, cv::Mat input_img_3D, bool arg_display, bool arg_save, std::string nameFile_toSave="/tmp");
      void stereovision_hybrid(cv::Mat fundamental_matrix, Eigen::MatrixXd projection_matrix, struct Input_Points input_points, struct Input_Images input_img, struct Output_Points& output_points, bool arg_display, bool arg_save, std::string calibration_namefile, std::string nameFile_toSave="/tmp");
};

#endif
