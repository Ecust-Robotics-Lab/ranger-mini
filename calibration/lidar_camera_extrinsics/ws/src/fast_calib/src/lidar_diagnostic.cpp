#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
int main(int argc,char** argv) {
 rclcpp::Node::SharedPtr n;
 Params p{};bool offline=argc>1 && std::string(argv[1])=="--offline";
 if (offline) {
   if(argc!=14){std::cerr<<"--offline BAG OUT xmin xmax ymin ymax zmin zmax radius width height frames\n";return 2;}
   p.lidar_topic="/livox/lidar";p.image_topic="/front/image_sync";p.bag_path=argv[2];p.output_path=argv[3];p.x_min=std::stod(argv[4]);p.x_max=std::stod(argv[5]);p.y_min=std::stod(argv[6]);p.y_max=std::stod(argv[7]);p.z_min=std::stod(argv[8]);p.z_max=std::stod(argv[9]);p.circle_radius=std::stod(argv[10]);p.delta_width_circles=std::stod(argv[11]);p.delta_height_circles=std::stod(argv[12]);p.max_frames=std::stoi(argv[13]);
 } else {rclcpp::init(argc,argv);n=std::make_shared<rclcpp::Node>("fast_calib");p=loadParameters(n);}
 std::filesystem::create_directories(p.output_path);
 DataPreprocess d(p);LidarDetect ld(n,p);pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
 if(d.lidar_type_!=LiDARType::Solid){std::cerr<<"Expected solid-state cloud (no ring)\n";return 2;}
 ld.detect_solid_lidar(d.cloud_input_,c);
 auto save=[&](auto name,auto cloud){if(!cloud->empty())pcl::io::savePCDFileBinary(p.output_path+"/"+name,*cloud);};
 save("roi.pcd",ld.getFilteredCloud());save("plane.pcd",ld.getPlaneCloud());save("aligned.pcd",ld.getAlignedCloud());save("edges.pcd",ld.getEdgeCloud());save("circle_candidates.pcd",ld.getCenterZ0Cloud());save("centers.pcd",c);
 auto plane=ld.getPlaneCloud();double rms=-1,distance=-1;
 if(plane->size()>=3){Eigen::Vector4f mu;pcl::compute3DCentroid(*plane,mu);Eigen::Matrix3f cov;pcl::computeCovarianceMatrixNormalized(*plane,mu,cov);Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);rms=std::sqrt(std::max(0.f,es.eigenvalues()[0]));distance=std::abs(es.eigenvectors().col(0).dot(mu.head<3>()));}
 std::ofstream o(p.output_path+"/lidar_quality.json");o<<"{\"input_points\":"<<d.cloud_input_->size()<<",\"roi_points\":"<<ld.getFilteredCloud()->size()<<",\"plane_points\":"<<plane->size()<<",\"edge_points\":"<<ld.getEdgeCloud()->size()<<",\"centers\":"<<c->size()<<",\"plane_rms_m\":"<<rms<<",\"plane_distance_m\":"<<distance<<",\"feature_match_verified\":false,\"geometry_accepted\":false}";
 if (!offline) { rclcpp::shutdown(); }
 return c->size()==4?0:3;
}
