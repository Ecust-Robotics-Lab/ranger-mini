#include "qr_detect.hpp"
#define main unused_upstream_multi_main
#include "multi_scene.cpp"
#undef main
int main() {
 // Synthetic geometry only: do not confuse this with actual calibration accuracy.
 Params p{};p.fx=1300;p.fy=1300;p.cx=640;p.cy=512;p.k3=-0.1;p.marker_size=.2;p.delta_width_qr_center=.55;p.delta_height_qr_center=.35;p.delta_width_circles=.5;p.delta_height_circles=.4;p.min_detected_markers=4;
 QRDetect qr(nullptr,p);
 if(std::abs(qr.distCoeffs_.at<float>(0,4)-p.k3)>1e-6)return 1;
 PointCloud<PointXYZ>::Ptr l(new PointCloud<PointXYZ>),c(new PointCloud<PointXYZ>),ls(new PointCloud<PointXYZ>),cs(new PointCloud<PointXYZ>);
 for (auto xy : std::vector<std::pair<float,float>>{{-.25,.2},{.25,.2},{.25,-.2},{-.25,-.2}}) {
  l->push_back(PointXYZ(2,xy.first,xy.second)); c->push_back(PointXYZ(-xy.first,-xy.second,2));
 }
 sortPatternCenters(l,ls,"lidar");sortPatternCenters(c,cs,"camera");
 for(int i=0;i<4;i++)if(std::abs((*cs)[i].x+(*ls)[i].y)>1e-6 || std::abs((*cs)[i].y+(*ls)[i].z)>1e-6)return 2;
 std::vector<std::vector<int>> groups;comb(3,4,groups);if(!groups.empty())return 3;
 groups.clear();comb(8,4,groups);if(groups.size()!=70)return 4;
 cv::Mat blank(1024,1280,CV_8UC3,cv::Scalar(255,255,255));PointCloud<PointXYZ>::Ptr empty(new PointCloud<PointXYZ>);qr.detect_qr(blank,empty);if(!empty->empty())return 5;
 p.k3=0;QRDetect synthetic(nullptr,p);
 auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
 cv::Mat rendered(1024,1280,CV_8UC3,cv::Scalar(255,255,255));
 std::vector<int> ids{1,2,4,3};std::vector<std::pair<float,float>> signs{{-1,1},{1,1},{1,-1},{-1,-1}};
 for(int i=0;i<4;i++) {
   cv::Mat marker,marker3,warped;cv::aruco::drawMarker(dict,ids[i],200,marker);cv::cvtColor(marker,marker3,cv::COLOR_GRAY2BGR);
   float x=signs[i].first*.55,y=signs[i].second*.35;std::vector<cv::Point2f> dst;
   for(auto dxy:std::vector<std::pair<float,float>>{{-.1,.1},{.1,.1},{.1,-.1},{-.1,-.1}}) dst.push_back(cv::Point2f(1300*(x+dxy.first)/2.5+640,-1300*(y+dxy.second)/2.5+512));
   std::vector<cv::Point2f> src{{0,0},{199,0},{199,199},{0,199}};
   cv::warpPerspective(marker3,warped,cv::getPerspectiveTransform(src,dst),rendered.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));cv::min(rendered,warped,rendered);
 }
 PointCloud<PointXYZ>::Ptr holes(new PointCloud<PointXYZ>);synthetic.detect_qr(rendered,holes);
 if(holes->size()!=4)return 6;
 for(const auto& h:*holes)if(std::abs(h.z-2.5)>.03)return 7;
 std::cout<<"SYNTHETIC_PNP_INTERFACE_OK (not calibration evidence)\n";
 std::vector<Eigen::Vector3d> lp,cp;
 Eigen::Matrix3d expectedR=Eigen::AngleAxisd(.2,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();Eigen::Vector3d expectedT(.1,-.2,.3);
 for(int scene=0;scene<3;scene++)for(const auto& h:*l) {Eigen::Vector3d v(h.x+scene*.1,h.y,h.z+scene*.3);lp.push_back(v);cp.push_back(expectedR*v+expectedT);}
 auto joint=SolveRigidTransformWeighted(lp,cp,nullptr);
 if(!joint.ok || (joint.R-expectedR).norm()>1e-9 || (joint.t-expectedT).norm()>1e-9 || joint.rms>1e-9)return 8;
 std::vector<Eigen::Vector3d> degenerate(12,Eigen::Vector3d(1,1,1));if(SolveRigidTransformWeighted(degenerate,degenerate,nullptr).ok)return 9;
 std::cout<<"SYNTHETIC_MULTI_SVD_INTERFACE_OK (not calibration evidence)\n";
 std::cout<<"OFFLINE_CHECKS_OK k3 preserved; Livox/optical sorting; missing features rejected\n";return 0;
}
