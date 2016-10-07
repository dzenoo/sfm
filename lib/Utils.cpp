/** 
 *  Copyright 2016
 *  @file    Utils.cpp
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
#include "Utils.h"

#include "masteringocv.h"


bool checkFundamentalMatrix(const std::vector<cv::Point2f>& imgpts1,
    const std::vector<cv::Point2f>& imgpts2, const cv::Mat& F) {
    std::ofstream myfile, myfile2;
    myfile.open("imgpts1.txt");
    myfile2.open("imgpts2.txt");

    myfile << "[";
    myfile2 << "[";
    for (unsigned int i = 0; i < imgpts1.size(); i++) {
        myfile << imgpts1[i].x << "," << imgpts1[i].y << ";\n";
        myfile2 << imgpts2[i].x << "," << imgpts2[i].y << ";\n";
        cv::Matx13d u1_i = cv::Matx13d(imgpts1[i].x, imgpts1[i].y, 1);
        cv::Matx31d u1_j = cv::Matx31d(imgpts2[i].x, imgpts2[i].y, 1);
        double rez = cv::Mat(cv::Mat(u1_i)*F*cv::Mat(u1_j)).at<double>(0, 0);
        std::cout << "Rez = " << rez << " - Tačka " << i << " - img1(" <<
                     imgpts1[i].x << "," << imgpts1[i].y << ") i img2(" <<
                     imgpts2[i].x << "," << imgpts2[i].y << ")" << std::endl;
        if (std::abs(rez) > 1e-7) std::cout << "LOSE" << std::endl;
    }
    myfile << "]";
    myfile.close();
    myfile2 << "]";
    myfile2.close();

    return true;
}

bool FindCameraMatricesAndPointCloud(const cv::Mat& K,
        const std::vector<cv::Point2f>& imgpts1_temp,
        const std::vector<cv::Point2f>& imgpts2_temp,
        const cv::Matx34d& P,
        cv::Matx34d& P1,
        std::vector<cv::Point3d>& pointcloud,
        std::vector<cv::Point2f>& imgpts1,
        std::vector<cv::Point2f>& imgpts2) {
    // Similar to FindCameraMatrices function from 
    // MasteringOpenCV but with some upgrades
    pointcloud.clear(); imgpts1.clear(); imgpts2.clear();
    std::vector<uchar> status(imgpts1_temp.size());

    double minVal, maxVal;
    cv::minMaxIdx(imgpts1_temp, &minVal, &maxVal);

    cv::Mat F = cv::findFundamentalMat(imgpts1_temp, imgpts2_temp, cv::FM_RANSAC, 0.006*maxVal, 0.99, status);

    // http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Mat_<double> E = cv::Mat(K.t()) * F * cv::Mat(K);  // according

    // clear the bad values in temporary vectors
    int sum = 0;
    for (unsigned int i=0; i < status.size(); i++)
        if (status[i]) {
            imgpts1.push_back(imgpts1_temp[i]);
            imgpts2.push_back(imgpts2_temp[i]);
            sum++;
        }
    // http://web.mit.edu/be.400/www/SVD/Singular_Value_Decomposition.htm

    // Check if the matrix E is valid
    // http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
    // according to http://en.wikipedia.org/wiki/Essential_matrix#Properties_of_the_essential_matrix
    if (fabsf(determinant(E)) > 1e-05) {
                std::cout << "det(E) != 0 : " << determinant(E) << "\n";
                P1 = 0;
                return false;
    }
    cv::Mat_<double> R1(3, 3);
    cv::Mat_<double> R2(3, 3);
    cv::Mat_<double> t1(1, 3);
    cv::Mat_<double> t2(1, 3);


    cv::Matx44d A_last = cv::Matx44d(P(0, 0), P(0, 1), P(0, 2), P(0, 3),
                                     P(1, 0), P(1, 1), P(1, 2), P(1, 3),
                                     P(2, 0), P(2, 1), P(2, 2), P(2, 3),
                                          0,        0,       0,      1);
    std::cout << "A_last: " << cv::Mat(A_last) << std::endl;
    cv::Matx44d A;
    cv::Matx44d A_temp;
    // Ambiguity of matrix P

    if (!DecomposeEtoRandT(E, R1, R2, t1, t2)) return false;

    if (determinant(R1)+1.0 < 1e-09) {
        // according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
        std::cout << "det(R) == -1 [" << determinant(R1) << "]: flip E's sign";
        E = -E;
        DecomposeEtoRandT(E, R1, R2, t1, t2);
    }
    if (!CheckCoherentRotation(R1)) {
        std::cout << "resulting rotation is not coherent\n";
        P1 = 0;
        return false;
    }
    /////////////////////////////////////////////////////
    A_temp = cv::Matx44d(R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
                         R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
                         R1(2, 0), R1(2, 1), R1(2, 2), t1(2),
                               0,         0,        0,    1);
    A = A_last * A_temp;

    P1 = cv::Matx34d(A(0, 0), A(0, 1), A(0, 2), A(0, 3),
                     A(1, 0), A(1, 1), A(1, 2), A(1, 3),
                     A(2, 0), A(2, 1), A(2, 2), A(2, 3));
    /////////////////////////////////////////////////////
    std::cout << "Testing P1 " << std::endl << cv::Mat(P1) << std::endl;

    std::vector<cv::Point3d> pcloud, pcloud1;
    double reproj_error1 = TriangulatePoints(imgpts1, imgpts2, K, P, P1, pcloud);
    double reproj_error2 = TriangulatePoints(imgpts2, imgpts1, K, P1, P, pcloud1);
    std::vector<uchar> tmp_status;
    //check if pointa are triangulated --in front-- of cameras for all 4 ambiguations
    if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
        /////////////////////////////////////////////////////
        A_temp = cv::Matx44d(R1(0, 0), R1(0, 1), R1(0, 2), t2(0),
                             R1(1, 0), R1(1, 1), R1(1, 2), t2(1),
                             R1(2, 0), R1(2, 1), R1(2, 2), t2(2),
                                   0,         0,        0,    1);
        A = A_last * A_temp;

        P1 = cv::Matx34d(A(0, 0), A(0, 1), A(0, 2), A(0, 3),
                         A(1, 0), A(1, 1), A(1, 2), A(1, 3),
                         A(2, 0), A(2, 1), A(2, 2), A(2, 3));
        /////////////////////////////////////////////////////
        std::cout << "Testing P1 "<< std::endl << cv::Mat(P1) << std::endl;

        pcloud.clear(); pcloud1.clear();
        reproj_error1 = TriangulatePoints(imgpts1, imgpts2, K, P, P1, pcloud);
        reproj_error2 = TriangulatePoints(imgpts2, imgpts1, K, P1, P, pcloud1);

        if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
            if (!CheckCoherentRotation(R2)) {
                std::cout << "resulting rotation is not coherent\n";
                P1 = 0;
                return false;
            }
            /////////////////////////////////////////////////////
            A_temp = cv::Matx44d(R2(0, 0), R2(0, 1), R2(0, 2), t1(0),
                                 R2(1, 0), R2(1, 1), R2(1, 2), t1(1),
                                 R2(2, 0), R2(2, 1), R2(2, 2), t1(2),
                                       0,         0,        0,    1);
            A = A_last * A_temp;

            P1 = cv::Matx34d(A(0, 0), A(0, 1), A(0, 2), A(0, 3),
                             A(1, 0), A(1, 1), A(1, 2), A(1, 3),
                             A(2, 0), A(2, 1), A(2, 2), A(2, 3));
            /////////////////////////////////////////////////////
            std::cout << "Testing P1 "<< std::endl << cv::Mat(P1) << std::endl;

            pcloud.clear(); pcloud1.clear();
            reproj_error1 = TriangulatePoints(imgpts1, imgpts2, K, P, P1, pcloud);
            reproj_error2 = TriangulatePoints(imgpts2, imgpts1, K, P1, P, pcloud1);

            if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                /////////////////////////////////////////////////////
                A_temp = cv::Matx44d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0),
                                     R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
                                     R2(2, 0), R2(2, 1), R2(2, 2), t2(2),
                                           0,         0,        0,    1);
                A = A_last * A_temp;

                P1 = cv::Matx34d(A(0, 0), A(0, 1), A(0, 2), A(0, 3),
                                 A(1, 0), A(1, 1), A(1, 2), A(1, 3),
                                 A(2, 0), A(2, 1), A(2, 2), A(2, 3));
                /////////////////////////////////////////////////////
                std::cout << "Testing P1 " << std::endl << cv::Mat(P1) << std::endl;

                pcloud.clear(); pcloud1.clear();
                reproj_error1 = TriangulatePoints(imgpts1, imgpts2, K, P, P1, pcloud);
                reproj_error2 = TriangulatePoints(imgpts2, imgpts1, K, P1, P, pcloud1);

                if (!TestTriangulation(pcloud, P1, tmp_status) || !TestTriangulation(pcloud1, P, tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0)
                    return false;
            }
        }
    }
    for (unsigned int i=0; i < pcloud.size(); i++) {
        pointcloud.push_back(pcloud[i]);
    }
    return true;
}

void GetBGRForPointCloud(const cv::Mat& img1, const cv::Mat& img2,
    const std::vector<cv::Point2f>& imgpts1,
    const std::vector<cv::Point2f>& imgpts2,
    std::vector<cv::Vec3b>& pointcloud_colors) {
    for (unsigned int i = 0; i < imgpts1.size(); i++){
        cv::Vec3b intensity1 = img1.at<cv::Vec3b>(imgpts1[i].y, imgpts1[i].x);
        cv::Vec3b intensity2 = img2.at<cv::Vec3b>(imgpts2[i].y, imgpts2[i].x);
        cv::Vec3b intensity = intensity1/2 + intensity2/2;
        pointcloud_colors.push_back(intensity);
    }
}
void exportPointCloud(const std::vector<cv::Vec3f>& pc,
    const std::vector<cv::Vec3b>& pc_colors, const QString& name) {
        QString filename = name;
        QFile file(filename);
        if (file.open(QIODevice::ReadWrite)) {
            QTextStream stream(&file);
            for (unsigned int i = 0; i < pc.size();i++)
                stream << pc[i][0]<< " "<< pc[i][1] << " " << pc[i][2] << " "
                       << pc_colors[i][2] << " " << pc_colors[i][1] << " "
                       << pc_colors[i][0] << " " << endl;
        }
}
