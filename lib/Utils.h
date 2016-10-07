/** 
 *  Copyright 2016
 *  @file    Utils.h
 *  @author  Dzenan Lapandic
 *  @date    8/8/2016  
 *  @version 1.0 
 *  
 *  @brief 
 *
 *  @section DESCRIPTION
 *  
 *
 */
#include <vector>

#include <iostream>
#include <fstream>
#include <string>
#include <set>

#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#include <QString>
#include <QFile>
#include <QTextStream>

#ifndef SFM_LIB_UTILS_H_
#define SFM_LIB_UTILS_H_

void GetBGRForPointCloud(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Point2f>& imgpts1, const std::vector<cv::Point2f>& imgpts2, std::vector<cv::Vec3b>& pointcloud_colors);

bool FindCameraMatricesAndPointCloud(const cv::Mat& K, const std::vector<cv::Point2f>& imgpts1_temp, const std::vector<cv::Point2f>& imgpts2_temp, const cv::Matx34d& P, cv::Matx34d& P1, std::vector<cv::Point3d>& pointcloud, std::vector<cv::Point2f>& imgpts1, std::vector<cv::Point2f>& imgpts2);

bool checkFundamentalMatrix(const std::vector<cv::Point2f>& imgpts1, const std::vector<cv::Point2f>& imgpts2, const cv::Mat& F);

void exportPointCloud(const std::vector<cv::Vec3f>& pc, const std::vector<cv::Vec3b>& pc_colors, const QString& naziv);

#endif  // SFM_LIB_UTILS_H_
