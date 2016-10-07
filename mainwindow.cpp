/** 
 *  Copyright 2016
 *  @file    mainwindow.cpp
 *  @author  Dzenan Lapandic
 *  @date    3/18/2016  
 *  @version 1.0 
 *  
 *  @brief Reconstruction of 3D structures from a sequence of 2D images
 *
 *  @section DESCRIPTION
 *  
 *
 */


#define CERES_FOUND true

#include "mainwindow.h"

#include <math.h>

#include <QFile>
#include <QTextStream>
#include <QDialog>
#include <QFileDialog>
#include <QElapsedTimer>
#include <QDebug>
#include <QTime>

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

#include "ui_mainwindow.h"

#include <lib/masteringocv.h>
#include <lib/Utils.h>

// #include <opencv2/xfeatures2d/nonfree.hpp>

// PCL include for Statistical outlier removal filter

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/statistical_outlier_removal.h>



QString dir_location;
QDir dir;
QStringList image_paths;
QString Qvideo_path;
bool read_video = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
        ui->setupUi(this);
    }

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_offlineButton_clicked() {
}

void MainWindow::on_openButton_clicked() {
    dir_location = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                    "/home",
                                                    QFileDialog::ShowDirsOnly
                                                    | QFileDialog::DontResolveSymlinks);
    ui->lineEdit->setText(dir_location);
    dir.setPath(dir_location);

    QFileInfoList files = dir.entryInfoList();

    image_paths = QStringList();
    int i = 0;
    foreach(QFileInfo f, files) {
        if (i<2) {
            i++; 
            continue;
        }
        if (f.completeSuffix() == "jpg" || f.completeSuffix() == "png" || f.completeSuffix() == "jpeg" || f.completeSuffix() == "JPG")
            image_paths.append(f.absoluteFilePath());
    }
    ui->listWidget->clear();
    ui->listWidget->addItems(image_paths);
}

void MainWindow::on_exportButton_clicked() {
}

void MainWindow::on_openButton2_clicked() {
    // Detect features and save new image with features
    bool use_surf = false;
    int surfTreshold = 600;
    double sr_vr_keypoints = 0;
    double sr_vr_time = 0;
    int fastTreshold = 10;
    int nMilliseconds = 0;
    for (int image_num = 1; image_num < 6; image_num++) {
        QTime my_timer2;
        my_timer2.start();

        std::string path0_i = "../../sfmdataset/oxford/images(3)/00"+std::to_string(image_num)+".jpg";
        cv::Mat frame = cv::imread(path0_i);
        cv::Mat img_i;
        // cv::Mat img_i_BGR = frame;
        cv::cvtColor(frame, img_i, cv::COLOR_BGR2GRAY);
        // FAST
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(fastTreshold);
        std::string image_name = "../../sfmdataset/output/oxford/FAST/fast" + std::to_string(image_num) + ".png";
        
        if (use_surf) {  // SURF
            detector = cv::xfeatures2d::SurfFeatureDetector::create(surfTreshold);
            image_name = "../../sfmdataset/output/oxford/SURF/surf" + std::to_string(image_num) + ".png";
        }
        std::vector<cv::KeyPoint> keypoints1;
        detector->detect(img_i, keypoints1);

        nMilliseconds = my_timer2.elapsed();
        std::cout << "Image number " << image_num << std::endl;
        std::cout << "Operation took: " << nMilliseconds << std::endl;
        std::cout << "Detected points: " << keypoints1.size() << std::endl;
        sr_vr_keypoints += keypoints1.size();
        sr_vr_time += nMilliseconds;
        cv::Mat new_img_i;
        cv::drawKeypoints(img_i,keypoints1, new_img_i);
        cv::imwrite(image_name, new_img_i);
    }
    sr_vr_keypoints = sr_vr_keypoints/5;
    sr_vr_time = sr_vr_time/5;
    std::cout << "Total time: " <<  sr_vr_time << std::endl;
    std::cout << "Total detected points: " << sr_vr_keypoints << std::endl;
}

void MainWindow::on_onlineButton_clicked() {
    QTime my_timer2;
    my_timer2.start();
    bool use_surf = false;

    int surfTreshold = 600;
    int fastTreshold = 10;
    // for video fastTreshold = 20

    cv::String video_path = Qvideo_path.toStdString();
    cv::VideoCapture video(video_path);
    int frame_id = 1;

    cv::Mat frame;
    int image_num = 0;

    if (read_video) {
        video >> frame;    
    } else {
        frame = cv::imread(image_paths.at(image_num).toStdString());
    }
    cv::Mat img_i;
    cv::Mat img_i_BGR = frame;
    cv::cvtColor(frame, img_i, cv::COLOR_BGR2GRAY);

    //std::string image_name1 = "../../sfmdataset/output/final" + std::to_string(1) + ".png";
    //cv::imwrite(image_name1,img_i_BGR);

    float cx = frame.cols / 2.0f;
    float cy = frame.rows / 2.0f;
    float f = frame.cols * 5.0f / 6.17f;

    cv::Matx33d K = cv::Matx33d( f, 0, cx,
                         0, f, cy,
                         0, 0,  1);

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    std::vector<cv::DMatch> allMatches;

    std::vector<cv::Point2f> i_pts; // detected points of image i
    std::vector<cv::Point2f> j_pts; // detected points of image j = i+1, used for output

    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(fastTreshold);
    if (use_surf) {
        // SURF Detector
        detector = cv::xfeatures2d::SurfFeatureDetector::create(surfTreshold);
    }
    detector->detect(img_i, keypoints1);

    // SURF Extractor
    // cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();
    // cv::Mat descriptors1;
    // extractor->compute(img_i, keypoints1, descriptors1);
    // detector->detect(img_i, keypoints1);

    std::vector<cv::Point2f>left_points;
    cv::KeyPoint::convert(keypoints1,left_points); // convert detected points of left image in Point2f
    std::vector<cv::Point2f>right_points(left_points.size());
    std::vector<cv::Matx34d> projectionMatrices;
    std::vector<cv::Matx34d> projectionMatrices_est;

    cv::Matx34d P0(1,0,0,0,
                   0,1,0,0,
                   0,0,1,0);

    projectionMatrices.push_back(P0);
    projectionMatrices_est.push_back(P0);

    /// Create 3D windows
    cv::viz::Viz3d window("Coordinate Frame");
               window.setWindowSize(cv::Size(500,500));
               window.setWindowPosition(cv::Point(150,150));
               window.setBackgroundColor(); // black by default

    std::vector<cv::Vec3f> point_cloud_est;
    std::vector<cv::Vec3b> point_cloud_est_colors;

    std::vector<cv::Affine3d> path;
    cv::Mat_<double> R0 = cv::Mat_<double>::eye(3,3);
    cv::Mat_<double> t0 = cv::Mat_<double>::zeros(3,1);

    path.push_back(cv::Affine3d(R0,t0));
    int point_cloud_itterator = 0;

    //bool KALMAN = false;// KALMAN FILTER, not finished!
    //bool SKIP = false;
    int frame_skip = 7;
    int number_of_images = image_paths.size(); // number of images in sequence
    if (read_video) number_of_images = 5; // number of frames to read from video
    std::vector<cv::Point3d> pointcloud;
    while(!window.wasStopped()){
        QTime my_timer;
        my_timer.start();
        std::vector<cv::DMatch> matches;
        std::vector<cv::Point3f> pointcloud_for_PNP;

        image_num++;
        if (image_num == number_of_images) {
            int nMilliseconds = my_timer2.elapsed();
            std::cout << "Elapsed time: " << nMilliseconds << std::endl;
            std::cout << "Pointcloud size: " << point_cloud_est.size() << std::endl;
            window.spin();
            break;
        }

        if (read_video) {
           for(int i = 1; i <= frame_skip + 1; i++)
            video >> frame;   
        } else {
            frame = cv::imread(image_paths.at(image_num).toStdString());
        }

        cv::Mat img_j_BGR = frame;
        cv::Mat img_j;
        cv::cvtColor(frame,img_j, cv::COLOR_BGR2GRAY);
        // std::string image_name = "../../sfmdataset/output/final" + std::to_string(image_num) + ".png";
        // cv::imwrite(image_name,img_j_BGR);

        detector->detect(img_j, keypoints2);
        frame_id++;
        
        std::vector<uchar>vstatus;
        std::vector<float>verror;
        // Calculate Optical Flow Lucas-Kanade method using openCV, patch size 15x15
        cv::calcOpticalFlowPyrLK(img_i, img_j, left_points, right_points, vstatus, verror, cv::Size(15,15));
        float mean_verror = cv::mean(verror)[0];
        std::vector<cv::Point2f> imgpts1_temp, imgpts2_temp;
        // First, filter out the points with high error
        for (unsigned int i=0; i < vstatus.size(); i++) {
            if (vstatus[i] && verror[i] < mean_verror) {
                if(right_points[i].x < 0 || right_points[i].y < 0){ 
                    continue;
                }
                if (frame_id > 2) {
                    pointcloud_for_PNP.push_back(pointcloud[i]);
                }
                imgpts1_temp.push_back(left_points[i]);
                imgpts2_temp.push_back(right_points[i]);
            } else {
                vstatus[i] = 0; // a bad flow
            }
         }
        cv::Matx34d P1;
        cv::Matx34d P_est;
        cv::Matx33d R;
        cv::Mat_<double> tvec,rvec,distCoeffs;
        pointcloud.clear();
        std::vector<cv::Point2f> imgpts1, imgpts2;
        if (frame_id < 3) {
            //generating first pointcloud, baseline triangulation
            pointcloud.clear();

            bool triang_error = FindCameraMatricesAndPointCloud(cv::Mat(K),imgpts1_temp, imgpts2_temp, projectionMatrices.back(), P1, pointcloud, imgpts1, imgpts2);

            if (!triang_error) {
                std::cout << "Triangulation Error" << std::endl;
                frame_id--;// skip this image, try with next
                continue;
            }
            P_est = P1;
        } else {
            // PnP estimation after the baseline triangulation
            cv::solvePnPRansac(pointcloud_for_PNP, imgpts2_temp, K, distCoeffs, rvec, tvec); 
            //pointcloud_for_PnP is pointcloud with baseline triangulation
            cv::Rodrigues(rvec,R);
            P1 = cv::Matx34d(R(0,0),R(0,1),R(0,2),tvec(0),
                             R(1,0),R(1,1),R(1,2),tvec(1),
                             R(2,0),R(2,1),R(2,2),tvec(2));
            std::vector<cv::KeyPoint> keypoints1_PNP;
            detector->detect(img_i, keypoints1_PNP);

            std::vector<cv::Point2f>left_points_PNP;

            cv::KeyPoint::convert(keypoints1_PNP,left_points_PNP);
            std::vector<cv::Point2f>right_points_PNP(left_points_PNP.size());

            std::vector<uchar>vstatus2;
            std::vector<float>verror2;

            cv::calcOpticalFlowPyrLK(img_i, img_j, left_points_PNP, right_points_PNP, vstatus2, verror2,cv::Size(15,15));

            float mean_verror2 = cv::mean(verror2)[0];
            std::vector<cv::Point2f> imgpts1_temp_PNP, imgpts2_temp_PNP;

            for (unsigned int i=0; i < vstatus2.size(); i++) {
                if (vstatus2[i] && verror2[i] < mean_verror2) {
                    if (right_points_PNP[i].x < 0 || right_points_PNP[i].y < 0) { 
                        continue;
                    }
                    imgpts1_temp_PNP.push_back(left_points_PNP[i]);
                    imgpts2_temp_PNP.push_back(right_points_PNP[i]);
                } else {
                    vstatus2[i] = 0; // a bad flow
                }
             }
            double error2 = TriangulatePoints(imgpts1_temp_PNP, imgpts2_temp_PNP, cv::Mat(K), projectionMatrices.back(), P1, pointcloud);
            std::cout << "Error of triangulation: " << error2 << std::endl;
            std::cout << "P1: " << cv::Mat(P1) << std::endl;
            imgpts1=imgpts1_temp_PNP; imgpts2=imgpts2_temp_PNP;
            P_est = P1;
        }

        std::vector<cv::Vec3b> pointcloud_colors;
        GetBGRForPointCloud(img_i_BGR,img_j_BGR,imgpts1,imgpts2,pointcloud_colors);

        //prepare for next iteration
        //SKIP = true;
        img_i = img_j;
        img_i_BGR = img_j_BGR;
        keypoints1 = keypoints2;
        //descriptors1 = descriptors2;
        cv::KeyPoint::convert(keypoints1,left_points);
        left_points = imgpts2;// only if using PNPransac!!!!!!!!!
        right_points.clear();
        right_points.resize(left_points.size());

        cv::Scalar pointcloud_mean = cv::mean(pointcloud);

        // recover estimated points3d
        for (unsigned int i = 0; i < pointcloud.size(); ++i){
            if(pointcloud[i].z < 0)
                continue;
          point_cloud_est.push_back(cv::Vec<double,3>(pointcloud[i]));
          point_cloud_est_colors.push_back(pointcloud_colors[i]);
        }

        // export pointcloud
        QString name_of_text_file = "pointcloud.txt";
        exportPointCloud(point_cloud_est, point_cloud_est_colors, name_of_text_file);

        // Recovering cameras
        std::cout << "Recovering cameras ... ";
        path.push_back(cv::Affine3d(cv::Matx33d(P_est(0, 0), P_est(0, 1), P_est(0, 2),
                                                P_est(1, 0), P_est(1, 1), P_est(1, 2),
                                                P_est(2, 0), P_est(2, 1), P_est(2, 2)),
                                    cv::Vec3d(P_est(0, 3), P_est(1, 3), P_est(2, 3))));
        // Add the pointcloud
        if (point_cloud_est.size() > 0 ) {
            std::cout << "Rendering points   ... ";
            cv::viz::WCloud cloud_widget(point_cloud_est, point_cloud_est_colors);
            window.showWidget("point_cloud", cloud_widget);
        } else {
            std::cout << "Cannot render points: Empty pointcloud" << std::endl;
        }

        // Add cameras
        if (path.size() > 0 ) {
              //cout << "Rendering Cameras  ... ";
              window.showWidget("cameras_frames_and_lines", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::green()));
              window.showWidget("cameras_frustums", cv::viz::WTrajectoryFrustums(path, K, 0.1, cv::viz::Color::yellow()));
              window.setViewerPose(cv::Affine3d(cv::Matx33d::eye(),cv::Vec3d(pointcloud_mean[0],pointcloud_mean[1],pointcloud_mean[2])));
        }
        else {
             std::cout << "Cannot render the cameras: Empty path" << std::endl;
        }

        this->setWindowTitle("Reconstruction of 3D structures from the 2D image sequences in real-time");

        point_cloud_itterator = point_cloud_est.size();
        std::string num_of_frames = "Reconstruction based on " + std::to_string(frame_id) + " images";
        window.showWidget("text2d", cv::viz::WText(num_of_frames, cv::Point(20, 20), 10, cv::viz::Color::white()));

        std::string num_of_points = "Pointcluod size: " + std::to_string(point_cloud_est.size()) + " points";
        window.showWidget("text2d2", cv::viz::WText(num_of_points, cv::Point(20, 10), 10, cv::viz::Color::white()));

        allMatches.insert(allMatches.end(), matches.begin(), matches.end());
        window.spinOnce(30,true);

        int nMilliseconds = my_timer.elapsed();
        std::cout << "Elapsed time: " << nMilliseconds << std::endl;
    }
}

void MainWindow::on_pushButton_4_clicked(){

}
void MainWindow::on_pushButton_clicked(){
    Qvideo_path = QFileDialog::getOpenFileName(this,
        tr("Open Video"), "/home", tr("Video Files (*.avi *.mov)"));
    read_video = true;
    ui->lineEdit_2->setText(Qvideo_path);
}
