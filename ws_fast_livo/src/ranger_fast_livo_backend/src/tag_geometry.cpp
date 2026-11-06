#include "ranger_fast_livo_backend/tags.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <algorithm>
#include <limits>

namespace ranger_backend::tags {
void require(bool c,const std::string& r){if(!c)throw std::runtime_error("tag_"+r);}
Pose pose(const Json& j){
  require(j.at("xyz").size()==3&&j.at("xyzw").size()==4,"pose_dimensions");
  Pose p=Pose::Identity();for(int k=0;k<3;++k)p.translation()[k]=j.at("xyz").at(k).get<double>();
  const auto& a=j.at("xyzw");Eigen::Quaterniond q(a[3].get<double>(),a[0].get<double>(),a[1].get<double>(),a[2].get<double>());
  require(q.coeffs().allFinite()&&std::abs(q.norm()-1)<1e-6,"pose_quaternion");p.linear()=q.normalized().toRotationMatrix();
  require(valid_pose(p)&&p.translation().norm()<1e6,"pose_invalid");return p;
}
Json read_json(const fs::path& p,size_t maximum){
  require(fs::is_regular_file(p)&&!fs::is_symlink(fs::symlink_status(p))&&fs::file_size(p)<=maximum,"json_file_or_size");
  std::ifstream in(p);std::map<int,std::set<std::string>> keys;
  return Json::parse(in,[&](int depth,Json::parse_event_t event,Json& v){
    require(depth<32,"json_depth");if(event==Json::parse_event_t::object_start)keys[depth+1].clear();
    if(event==Json::parse_event_t::key)require(keys[depth].insert(v.get<std::string>()).second,"duplicate_json_key");
    return true;
  });
}
namespace {
cv::Mat matrix(const Json& a,int rows,int cols){
  require(a.is_array()&&a.size()==size_t(rows*cols),"camera_matrix_dimensions");cv::Mat m(rows,cols,CV_64F);
  for(int i=0;i<rows*cols;++i){m.at<double>(i/cols,i%cols)=a.at(i).get<double>();require(std::isfinite(m.at<double>(i/cols,i%cols)),"camera_nonfinite");}return m;
}
Json array(const cv::Mat& m){Json a=Json::array();for(int r=0;r<m.rows;++r)for(int c=0;c<m.cols;++c)a.push_back(m.at<double>(r,c));return a;}
std::pair<cv::Mat,cv::Mat> rt(const Pose& T){cv::Mat R(3,3,CV_64F),t(3,1,CV_64F),r;for(int i=0;i<3;++i){t.at<double>(i)=T.translation()[i];for(int j=0;j<3;++j)R.at<double>(i,j)=T.linear()(i,j);}cv::Rodrigues(R,r);return {r,t};}
}
Camera Camera::parse(const Json& j){
  Camera c;c.width=j.at("width");c.height=j.at("height");c.domain=j.at("pixel_domain");c.model=j.at("distortion_model");c.frame=j.at("frame");
  require(c.width>0&&c.height>0&&int64_t(c.width)*c.height<=16000000&&!c.frame.empty(),"image_dimensions_or_frame");
  require(c.domain=="raw"||c.domain=="rectified"||c.domain=="normalized","pixel_domain");
  require(c.model=="plumb_bob"||c.model=="rational_polynomial"||c.model=="equidistant","unsupported_distortion_model");
  c.K=matrix(j.at("K"),3,3);c.D=matrix(j.at("D"),1,int(j.at("D").size()));
  c.R=matrix(j.value("R",Json::array({1,0,0,0,1,0,0,0,1})),3,3);
  require(c.K.at<double>(0,0)>0&&c.K.at<double>(1,1)>0&&std::abs(c.K.at<double>(2,2)-1)<1e-9&&
    std::abs(c.K.at<double>(0,1))+std::abs(c.K.at<double>(1,0))+std::abs(c.K.at<double>(2,0))+std::abs(c.K.at<double>(2,1))<1e-9,"invalid_intrinsics");
  require(cv::norm(c.R.t()*c.R-cv::Mat::eye(3,3,CV_64F))<1e-6&&std::abs(cv::determinant(c.R)-1)<1e-6,"rectification_rotation");
  require(j.value("binning_x",0)<=1&&j.value("binning_y",0)<=1&&j.value("roi_x",0)==0&&j.value("roi_y",0)==0&&
    j.value("roi_width",0)==0&&j.value("roi_height",0)==0,"roi_binning_requires_explicit_effective_model");
  require((c.model=="equidistant"&&c.D.total()==4)||(c.model=="plumb_bob"&&c.D.total()==5)||(c.model=="rational_polynomial"&&c.D.total()==8),"distortion_dimensions");
  if(c.domain!="raw")require(cv::norm(c.D)==0,"rectified_or_normalized_requires_zero_D");
  c.error_K=c.K;
  if(c.domain=="normalized"){
    require(cv::norm(c.K-cv::Mat::eye(3,3,CV_64F))<1e-9,"normalized_requires_identity_K");
    c.error_K=matrix(j.at("pixel_error_K"),3,3);
    require(c.error_K.at<double>(0,0)>0&&c.error_K.at<double>(1,1)>0,"normalized_pixel_error_scale");
  }
  return c;
}
Json Camera::json()const{Json j={{"width",width},{"height",height},{"pixel_domain",domain},{"distortion_model",model},{"frame",frame},{"K",array(K)},{"D",array(D)},{"R",array(R)}};if(domain=="normalized")j["pixel_error_K"]=array(error_K);return j;}
Config Config::parse(const Json& j){
  Config c;require(j.at("schema_version")==1,"config_schema");c.camera=Camera::parse(j.at("camera"));c.E=pose(j.at("T_imu_camera"));
  c.calibration_id=j.at("calibration_id");c.time_mapping_id=j.at("time_mapping_id");c.time_source=j.at("time_source");
  require(!c.calibration_id.empty()&&!c.time_mapping_id.empty()&&!c.time_source.empty(),"missing_provenance");
  c.offset_ns=j.value("offset_ns",int64_t(0));c.uncertainty_ns=j.value("uncertainty_ns",int64_t(0));c.time_verified=j.value("time_verified",false);
  require((!j.contains("offset_ns")||j.at("offset_ns").is_number_integer())&&(!j.contains("uncertainty_ns")||j.at("uncertainty_ns").is_number_integer()),"integer_time_mapping_required");
  require(c.offset_ns>=-1000000000LL&&c.offset_ns<=1000000000LL&&c.uncertainty_ns>=0&&c.uncertainty_ns<=1000000000LL,"time_mapping_bounds");
#define P(x) c.x=j.value(#x,c.x);require(std::isfinite(double(c.x))&&c.x>0,#x)
  P(pixel_rmse);P(pixel_max);P(min_area);P(history_seconds);P(history_gap);P(observation_age);P(maximum_wait);P(max_speed);P(max_angular_speed);
  P(mode_translation);P(mode_rotation);P(support_translation);P(support_rotation);P(history_capacity);P(observation_capacity);P(max_hypotheses);P(max_landmarks);P(min_support);
#undef P
  c.max_hamming=j.value("max_hamming",0);c.min_margin=j.value("min_margin",0.);
  for(const auto& entry:j.value("fixed_tags",Json::array())){
    std::string family=entry.at("family"),source=entry.at("source");int id=entry.at("id");
    require((family=="tag36h11"||family=="tagStandard41h12")&&id>=0&&!source.empty()&&entry.at("fixed")==true,"fixed_deployment_identity");
    require(c.fixed_tags.insert(family+":"+std::to_string(id)).second,"duplicate_fixed_deployment");
  }
  require(c.max_hamming>=0&&c.max_hamming<=2&&std::isfinite(c.min_margin)&&c.min_margin>=0,"decode_thresholds");
  require(c.history_capacity<=4096&&c.observation_capacity<=1024&&c.max_hypotheses<=32&&c.max_landmarks<=100000&&c.min_support>=2,"resource_limits");c.source=j;return c;
}
std::vector<cv::Point3d> object_points(double s){require(std::isfinite(s)&&s>0&&s<10,"size");return {{-s/2,s/2,0},{s/2,s/2,0},{s/2,-s/2,0},{-s/2,-s/2,0}};}
std::vector<cv::Point2d> project(const Camera& c,double s,const Pose& T){
  require(valid_pose(T),"projection_pose");auto points=object_points(s);Pose effective=T;
  // Rectified camera rotation maps original optical rays into rectified rays.
  if(c.domain=="rectified") {Eigen::Matrix3d R;for(int i=0;i<3;++i)for(int j=0;j<3;++j)R(i,j)=c.R.at<double>(i,j);effective.linear()=R*T.linear();effective.translation()=R*T.translation();}
  for(const auto& p:points)require((effective*Eigen::Vector3d(p.x,p.y,p.z)).z()>1e-6,"nonpositive_depth");
  auto [r,t]=rt(effective);std::vector<cv::Point2d> out;
  if(c.model=="equidistant"&&c.domain=="raw")cv::fisheye::projectPoints(points,out,r,t,c.K,c.D);
  else cv::projectPoints(points,r,t,c.K,c.D,out);
  for(const auto& p:out)require(std::isfinite(p.x)&&std::isfinite(p.y),"nonfinite_projection");
  return out;
}
std::pair<double,double> residual(const Camera& c,const Observation& o,const Pose& T){
  auto predicted=project(c,o.size,T);require(o.corners.size()==4,"corner_count");double sum=0,maximum=0;
  for(int k=0;k<4;++k){auto error=predicted[k]-o.corners[k];
    if(c.domain=="normalized"){error.x*=c.error_K.at<double>(0,0);error.y*=c.error_K.at<double>(1,1);}
    double d=cv::norm(error);sum+=d*d;maximum=std::max(maximum,d);}return {std::sqrt(sum/4),maximum};
}
std::vector<Hypothesis> solve(const Camera& c,double s,const std::vector<cv::Point2d>& corners){
  require(corners.size()==4,"corner_count");for(const auto& p:corners)require(std::isfinite(p.x)&&std::isfinite(p.y),"nonfinite_corner");
  std::vector<cv::Point2d> normalized;
  if(c.domain=="normalized")normalized=corners;
  else if(c.model=="equidistant"&&c.domain=="raw")cv::fisheye::undistortPoints(corners,normalized,c.K,c.D);
  else cv::undistortPoints(corners,normalized,c.K,c.D);
  std::vector<cv::Mat> rv,tv;cv::solvePnPGeneric(object_points(s),normalized,cv::Mat::eye(3,3,CV_64F),cv::noArray(),rv,tv,false,cv::SOLVEPNP_IPPE_SQUARE);
  std::vector<Hypothesis> out;Observation o;o.size=s;o.corners=corners;
  for(size_t h=0;h<std::min<size_t>(2,rv.size());++h)try {
    cv::Mat R;cv::Rodrigues(rv[h],R);Pose T=Pose::Identity();for(int i=0;i<3;++i){T.translation()[i]=tv[h].at<double>(i);for(int j=0;j<3;++j)T.linear()(i,j)=R.at<double>(i,j);}
    if(c.domain=="rectified"){Eigen::Matrix3d r;for(int i=0;i<3;++i)for(int j=0;j<3;++j)r(i,j)=c.R.at<double>(i,j);T.translation()=r.transpose()*T.translation();T.linear()=r.transpose()*T.linear();}
    require(valid_pose(T),"invalid_pnp_pose");auto [r,m]=residual(c,o,T);bool duplicate=false;
    for(const auto& v:out)if((v.T.translation()-T.translation()).norm()<1e-7&&rotation_distance(v.T,T)<1e-7)duplicate=true;
    if(!duplicate)out.push_back({T,int(h),r,m});
  }catch(const std::exception&){}return out;
}
Json Observation::json()const {
  Json pixels=Json::array(),poses=Json::array();for(auto p:corners)pixels.push_back({p.x,p.y});
  for(auto h:hypotheses)poses.push_back({{"hypothesis_id",h.id},{"T_camera_tag",pose_json(h.T)},{"pixel_rmse",h.rmse},{"pixel_max",h.maximum}});
  return {{"schema_version",1},{"observation_id",observation_id},{"image_id",image_id},{"source_id",source_id},{"context",context},
    {"family",family},{"id",id},{"size_m",size},{"canonical_frame",frame_convention},{"image_stamp_ns",raw_stamp},{"observation_stamp_ns",stamp},
    {"calibration_id",calibration_id},{"time_mapping_id",time_mapping_id},{"hamming",hamming},{"decision_margin",margin},
    {"corners",pixels},{"hypotheses",poses},{"provenance",provenance},{"quality",quality}};
}
Observation Observation::parse(const Json& j,const Config& c){
  Observation o;require(j.at("schema_version")==1&&j.at("canonical_frame")==frame_convention,"observation_schema_or_frame");
  o.observation_id=j.at("observation_id");o.image_id=j.at("image_id");o.source_id=j.at("source_id");o.context=j.at("context");
  for(auto s:{o.observation_id,o.image_id,o.source_id,o.context})require(!s.empty()&&s.size()<=512,"observation_identity");
  o.family=j.at("family");o.id=j.at("id");o.size=j.at("size_m");require((o.family=="tag36h11"||o.family=="tagStandard41h12")&&o.id>=0&&o.id<100000,"family_or_id");
  require(std::isfinite(o.size)&&o.size>=0&&o.size<10,"observation_size");
  require(j.at("image_stamp_ns").is_number_integer()&&j.at("observation_stamp_ns").is_number_integer(),"integer_nanoseconds_required");
  o.raw_stamp=j.at("image_stamp_ns");o.stamp=j.at("observation_stamp_ns");
  require(o.raw_stamp>1000000000LL&&o.raw_stamp<INT64_MAX-1000000000LL&&o.stamp==o.raw_stamp+c.offset_ns,"time_mapping_or_repeated_offset");
  o.calibration_id=j.at("calibration_id");o.time_mapping_id=j.at("time_mapping_id");
  require(o.calibration_id==c.calibration_id&&o.time_mapping_id==c.time_mapping_id,"calibration_or_time_mapping_generation");
  o.hamming=j.at("hamming");o.margin=j.at("decision_margin");require(o.hamming>=0&&o.hamming<=c.max_hamming&&std::isfinite(o.margin)&&o.margin>=c.min_margin,"decode_quality");
  require(j.at("corners").size()==4,"corner_count");for(const auto& p:j.at("corners")){require(p.size()==2,"corner_dimensions");double x=p[0],y=p[1];require(std::isfinite(x)&&std::isfinite(y),"corner_nonfinite");o.corners.emplace_back(x,y);}
  if(c.camera.domain!="normalized"){
    std::vector<cv::Point2f> polygon;for(auto p:o.corners){require(p.x>=0&&p.y>=0&&p.x<c.camera.width&&p.y<c.camera.height,"corner_outside_image");polygon.emplace_back(p.x,p.y);}
    require(cv::isContourConvex(polygon)&&std::abs(cv::contourArea(polygon))>=c.min_area,"corner_area_or_winding");
  }
  require(j.at("hypotheses").is_array()&&j.at("hypotheses").size()<=2,"hypothesis_limit");
  std::set<int> ids;
  for(const auto& h:j.at("hypotheses")){
    Hypothesis v;v.id=h.at("hypothesis_id");require(ids.insert(v.id).second,"duplicate_hypothesis");v.T=pose(h.at("T_camera_tag"));
    auto [r,m]=residual(c.camera,o,v.T);v.rmse=r;v.maximum=m;
    if(r<=c.pixel_rmse&&m<=c.pixel_max)o.hypotheses.push_back(v);
  }
  o.provenance=j.at("provenance");o.quality=j.at("quality");
  require(o.provenance.contains("size_source")&&o.provenance.contains("image_reference")&&o.provenance.contains("camera_model"),"observation_provenance");
  require(o.provenance.at("camera_model")==c.camera.json(),"observation_pixel_model_mismatch");
  return o;
}
void History::push(int64_t stamp,const Pose& U){
  require(stamp>0&&valid_pose(U),"history_input");
  // Out-of-order insertion is bounded; a clock reset is explicit at the owner.
  if(!samples_.empty())require(stamp>=samples_.back().stamp-int64_t(c_.history_seconds*1e9),"history_too_old");
  auto i=std::lower_bound(samples_.begin(),samples_.end(),stamp,[](const auto& s,int64_t t){return s.stamp<t;});
  require(i==samples_.end()||i->stamp!=stamp,"history_duplicate");samples_.insert(i,{stamp,U});
  while(samples_.size()>c_.history_capacity||(!samples_.empty()&&samples_.back().stamp-samples_.front().stamp>int64_t(c_.history_seconds*1e9)))samples_.pop_front();
}
std::optional<Pose> History::at(int64_t stamp)const {
  auto hi=std::lower_bound(samples_.begin(),samples_.end(),stamp,[](const auto& s,int64_t t){return s.stamp<t;});
  if(hi!=samples_.end()&&hi->stamp==stamp)return hi->U;
  if(hi==samples_.begin()||hi==samples_.end())return std::nullopt;
  auto lo=std::prev(hi);double dt=(hi->stamp-lo->stamp)*1e-9;
  if(dt>c_.history_gap||(hi->U.translation()-lo->U.translation()).norm()/dt>c_.max_speed||rotation_distance(lo->U,hi->U)/dt>c_.max_angular_speed)return std::nullopt;
  double alpha=double(stamp-lo->stamp)/double(hi->stamp-lo->stamp);Pose U=Pose::Identity();U.translation()=(1-alpha)*lo->U.translation()+alpha*hi->U.translation();
  U.linear()=Eigen::Quaterniond(lo->U.linear()).slerp(alpha,Eigen::Quaterniond(hi->U.linear())).normalized().toRotationMatrix();return U;
}
Json History::json()const{Json out=Json::array();for(const auto& s:samples_)out.push_back({{"stamp_ns",s.stamp},{"T_local_imu",pose_json(s.U)}});return out;}
void Buffer::reset(const std::string& context){context_=context;history_.clear();observations_.clear();seen_.clear();diagnostics_={{"context",context}};}
void Buffer::add_pose(int64_t stamp,const Pose& U,const std::string& context){if(context_!=context)reset(context);history_.push(stamp,U);}
void Buffer::push(const Json& batch,const std::string& context){
  require(batch.at("schema_version")==1&&batch.at("context")==context,"stale_context");
  if(context_!=context)reset(context);
  require(batch.at("observations").is_array()&&batch.at("observations").size()<=32,"batch_capacity");
  std::vector<Observation> incoming;std::set<std::string> keys,duplicates;
  for(const auto& j:batch.at("observations")){auto o=Observation::parse(j,c_);require(o.context==context,"stale_observation_context");if(!keys.insert(o.image_id+o.key()).second)duplicates.insert(o.image_id+o.key());incoming.push_back(std::move(o));}
  for(auto& o:incoming){
    if(duplicates.count(o.image_id+o.key())){diagnostics_["reason"]="duplicate_physical_id_in_image";continue;}
    auto identity=o.image_id+":"+o.key();
    if(std::find(seen_.begin(),seen_.end(),identity)!=seen_.end())continue;
    seen_.push_back(identity);if(seen_.size()>c_.observation_capacity*4)seen_.pop_front();
    if(observations_.size()>=c_.observation_capacity){observations_.pop_front();diagnostics_["reason"]="observation_capacity_drop";}
    o.received_steady=steady_seconds();observations_.push_back(std::move(o));
  }
}
std::vector<Attached> Buffer::take(int64_t stamp,const Pose& U){
  std::vector<Attached> out;
  for(auto i=observations_.begin();i!=observations_.end();){
    if(steady_seconds()-i->received_steady>c_.maximum_wait){diagnostics_["reason"]="observation_wait_timeout";i=observations_.erase(i);continue;}
    if(i->stamp>stamp){++i;continue;}
    auto Uc=history_.at(i->stamp);
    if(stamp-i->stamp>int64_t(c_.observation_age*1e9)){diagnostics_["reason"]="observation_expired";i=observations_.erase(i);continue;}
    if(!c_.time_verified){diagnostics_["reason"]="time_semantics_unverified_record_only";i=observations_.erase(i);continue;}
    if(!Uc){diagnostics_["reason"]="history_not_bracketed_or_motion_gap";i=observations_.erase(i);continue;}
    if(c_.uncertainty_ns>int64_t(c_.history_gap*1e9)){diagnostics_["reason"]="time_uncertainty_excessive";i=observations_.erase(i);continue;}
    if(!i->hypotheses.empty())out.push_back({*i,*Uc,U,U.inverse()*(*Uc)*c_.E,stamp});
    i=observations_.erase(i);
  }
  diagnostics_["pending"]=observations_.size();diagnostics_["history_samples"]=history_.size();diagnostics_["attached"]=out.size();return out;
}
}
