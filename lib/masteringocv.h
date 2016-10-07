/*****************************************************************************
*   ExploringSfMWithOpenCV
******************************************************************************
*   by Roy Shilkrot, 5th Dec 2012
*   http://www.morethantechnical.com/
******************************************************************************
*   Ch4 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
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

#ifndef SFM_MOPENCV_MASTERINGOCV_H_
#define SFM_MOPENCV_MASTERINGOCV_H_

bool DecomposeEtoRandT(cv::Mat_<double>& E, cv::Mat_<double>& R1, cv::Mat_<double>& R2, cv::Mat_<double>& t1, cv::Mat_<double>& t2);

bool TestTriangulation(const std::vector<cv::Point3d>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status);

bool CheckCoherentRotation(cv::Mat_<double>& R);

cv::Mat_<double> LinearLSTriangulation(const cv::Point3d& u, const cv::Matx34d& P, const cv::Point3d& u1, const cv::Matx34d& P1);

double TriangulatePoints( const std::vector<cv::Point2f>& pt_set1, const std::vector<cv::Point2f>& pt_set2, const cv::Mat& K, const cv::Matx34d& P, const cv::Matx34d& P1, std::vector<cv::Point3d>& pointcloud);

#endif // SFM_MOPENCV_MASTERINGOCV_H_
