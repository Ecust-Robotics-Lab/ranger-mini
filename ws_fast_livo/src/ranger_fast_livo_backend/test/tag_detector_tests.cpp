#include "tag_fixture.hpp"
#include <ranger_fast_livo_backend/tag_detector.hpp>
#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>
using namespace tag_fixture;
TEST(TagDetector,OfficialCodeRotationsPreserveDecodedCornerOrder){
  auto j=configuration();j["camera"]["D"]={0,0,0,0,0};auto c=t::Config::parse(j);t::Detector detector(c,"tag36h11",{{"0",.4}});
  auto tag=detector.render(0,20);cv::Mat scene(480,640,CV_8UC1,cv::Scalar(255));tag.copyTo(scene(cv::Rect(220,140,tag.cols,tag.rows)));
  // v3.4.5 tag36h11 renderer includes one white cell around an 8-cell border.
  // Native p0 is bottom-left, p1 bottom-right, p2 top-right, p3 top-left.
  std::vector<cv::Point2d> truth={{240,320},{400,320},{400,160},{240,160}};
  for(int turn=0;turn<4;++turn){double angle=turn*90.;auto affine=cv::getRotationMatrix2D({320,240},angle,1);cv::Mat rotated;
    cv::warpAffine(scene,rotated,affine,scene.size(),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255));
    auto batch=detector.detect(rotated,10000000000LL+turn,"session","image"+std::to_string(turn),"synthetic official code");
    ASSERT_EQ(batch["observations"].size(),1U)<<turn;auto o=t::Observation::parse(batch["observations"][0],c);
  EXPECT_EQ(o.id,0);
  ASSERT_FALSE(o.hypotheses.empty());
    for(int k=0;k<4;++k){double x=affine.at<double>(0,0)*truth[k].x+affine.at<double>(0,1)*truth[k].y+affine.at<double>(0,2),y=affine.at<double>(1,0)*truth[k].x+affine.at<double>(1,1)*truth[k].y+affine.at<double>(1,2);
  EXPECT_LT(cv::norm(o.corners[k]-cv::Point2d(x,y)),2.)<<turn<<" corner "<<k;}
  }
}
TEST(TagDetector,StrideEncodingUnknownSizeAndFamily){
  auto c=t::Config::parse(configuration());t::Detector detector(c,"tagStandard41h12",rb::Json::object());
  cv::Mat backing(482,660,CV_8UC1,cv::Scalar(255));auto view=backing(cv::Rect(3,1,640,480));
  ASSERT_FALSE(view.isContinuous());
  auto tag=detector.render(1,15);tag.copyTo(view(cv::Rect(200,150,tag.cols,tag.rows)));
  auto batch=detector.detect(view,10000000000LL,"session","image","synthetic");
  ASSERT_EQ(batch["observations"].size(),1U);
  auto o=t::Observation::parse(batch["observations"][0],c);
  EXPECT_EQ(o.family,"tagStandard41h12");
  EXPECT_EQ(o.size,0);
  EXPECT_TRUE(o.hypotheses.empty());
  auto gray=t::gray_image(640,480,view.step,"mono8",view.data,view.step*480);
  EXPECT_EQ(cv::norm(gray-view),0);
  EXPECT_THROW(t::gray_image(640,480,639,"mono8",view.data,view.step*480),std::exception);
  EXPECT_THROW(t::gray_image(640,480,view.step,"bayer",view.data,view.step*480),std::exception);
  EXPECT_THROW(t::gray_image(640,480,view.step,"mono8",view.data,10),std::exception);
  cv::Mat rgb;cv::cvtColor(view,rgb,cv::COLOR_GRAY2RGB);auto converted=t::gray_image(640,480,rgb.step,"rgb8",rgb.data,rgb.total()*3);
  EXPECT_EQ(cv::norm(converted-view),0);
}
TEST(TagDetector,SameImageDuplicatePhysicalIDIsRejected){
  auto c=t::Config::parse(configuration());t::Detector detector(c,"tag36h11",{{"0",.4}});cv::Mat scene(480,640,CV_8UC1,cv::Scalar(255));auto tag=detector.render(0,12);
  tag.copyTo(scene(cv::Rect(80,150,tag.cols,tag.rows)));tag.copyTo(scene(cv::Rect(410,150,tag.cols,tag.rows)));auto batch=detector.detect(scene,10000000000LL,"session","image","synthetic");
  ASSERT_EQ(batch["observations"].size(),2U);t::Buffer buffer(c);buffer.add_pose(10000000000LL,pose(),"session");buffer.push(batch,"session");
  EXPECT_TRUE(buffer.take(10000000000LL,pose()).empty());
}
