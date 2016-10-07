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
#include "masteringocv.h"

bool CheckCoherentRotation(cv::Mat_<double>& R) {
    if(std::fabs(std::fabs(cv::determinant(R))-1.0) > 1e-07) {
        std::cout<<"det(R) != +-1.0, this is not a rotation matrix"<<std::endl;
        return false;
    }
    return true;
}


cv::Mat_<double> LinearLSTriangulation(const cv::Point3d& u,//homogenous image point (u,v,1)
                                       const cv::Matx34d& P,//camera 1 matrix
                                       const cv::Point3d& u1,//homogenous image point in 2nd camera
                                       const cv::Matx34d& P1//camera 2 matrix
                                       )
{
    //build A matrix
    cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2), u.y*P(2,0)-P(1,0),
                  u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2), u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),
                  u1.x*P1(2,2)-P1(0,2), u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2) );

    //build B vector
    cv::Matx41d B(-(u.x*P(2,3)-P(0,3)), -(u.y*P(2,3)-P(1,3)), -(u1.x*P1(2,3)-P1(0,3)), -(u1.y*P1(2,3)-P1(1,3)));
    //solve for X
    cv::Mat_<double> X;
    cv::solve(A,B,X,cv::DECOMP_SVD);
    return X;

}

double TriangulatePoints( const std::vector<cv::Point2f>& pt_set1, const std::vector<cv::Point2f>& pt_set2, const cv::Mat& K, const cv::Matx34d& P, const cv::Matx34d& P1,
std::vector<cv::Point3d>& pointcloud) {
    cv::Mat Kinv = K.inv();
    std::vector<double> reproj_error;
    for (unsigned int i=0; i<pt_set1.size(); i++) {
        //convert to normalized homogeneous coordinates
        cv::Point2f kp = pt_set1[i];
        cv::Point3d u(kp.x,kp.y,1.0);

        cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
        u = um.at<cv::Point3d>(0);

        cv::Point2f kp1 = pt_set2[i];
        cv::Point3d u1(kp1.x,kp1.y,1.0);

        cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
        u1 = um1.at<cv::Point3d>(0);

        //triangulate
        cv::Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
        cv::Mat_<double> Xh = cv::Mat_<double>(4,1);
        Xh.at<double>(0) = X(0);
        Xh.at<double>(1) = X(1);
        Xh.at<double>(2) = X(2);
        Xh.at<double>(3) = 1;
    //calculate reprojection error
        cv::Mat_<double> xPt_img = cv::Mat(K) * cv::Mat(P1) * cv::Mat(Xh);
        cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
        double error = cv::norm(xPt_img_ - kp1);
        //if(error > 100)   continue;
        reproj_error.push_back(error);
        //store 3D point
        pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));
    }
    //return mean reprojection error
    cv::Scalar me = cv::mean(reproj_error);
    std::cout << "reproj_error " << me[0] << std::endl;
    return me[0];
}

bool TestTriangulation(const std::vector<cv::Point3d>& pcloud, const cv::Matx34d& P, std::vector<uchar>& status) {
    std::vector<cv::Point3d> pcloud_pt3d = pcloud;
    std::vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

    cv::Matx44d P4x4 = cv::Matx44d::eye();
    for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];

    cv::perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

    status.resize(pcloud.size(),0);
    for (unsigned int i=0; i<pcloud.size(); i++) {
        status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
    }
    int count = cv::countNonZero(status);

    double percentage = ((double)count / (double)pcloud.size());
    std::cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << std::endl;
    if(percentage < 0.70)
        return false; //less than 75% of the points are in front of the camera

    //check for coplanarity of points
    if(false) //not
    {
        cv::Mat_<double> cldm(pcloud.size(),3);
        for(unsigned int i=0;i<pcloud.size();i++) {
            cldm.row(i)(0) = pcloud[i].x;
            cldm.row(i)(1) = pcloud[i].y;
            cldm.row(i)(2) = pcloud[i].z;
        }
        cv::Mat_<double> mean;
        cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

        int num_inliers = 0;
        cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
        cv::Vec3d x0 = pca.mean;
        double p_to_plane_thresh = std::sqrt(pca.eigenvalues.at<double>(2));

        for (unsigned int i=0; i<pcloud.size(); i++) {
            cv::Vec3d w = cv::Vec3d(pcloud[i]) - x0;
            double D = fabs(nrm.dot(w));
            if(D < p_to_plane_thresh) num_inliers++;
        }

        std::cout << num_inliers << "/" << pcloud.size() << " are coplanar" << std::endl;
        if((double)num_inliers / (double)(pcloud.size()) > 0.85)
            return false;
    }

    return true;
}

bool DecomposeEtoRandT(
    cv::Mat_<double>& E,
    cv::Mat_<double>& R1,
    cv::Mat_<double>& R2,
    cv::Mat_<double>& t1,
    cv::Mat_<double>& t2)
{

    //Using HZ E decomposition

    cv::SVD svd(E,cv::SVD::MODIFY_A);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
    if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
        std::cout << "singular values are too far apart\n";
        return false;
    }

    cv::Matx33d W(0,-1,0,   //HZ 9.13
        1,0,0,
        0,0,1);
    cv::Matx33d Wt(0,1,0,
        -1,0,0,
        0,0,1);
    R1 = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
    R2 = svd.u * cv::Mat(Wt) * svd.vt; //HZ 9.19
    t1 = svd.u.col(2); //u3
    t2 = -svd.u.col(2); //u3

    return true;
}
