// Compile twice with ORIGINAL_MAPPER and the pre-refactor source, then without
// it against inspection_policy. Compare the output text and .bt files bytewise.
#include <opencv2/opencv.hpp>
#include <octomap/ColorOcTree.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#ifdef ORIGINAL_MAPPER
#include "realtime_stereo_mapper.hpp"
#else
#include "auv_inspection_planner/use_cases/mapping.hpp"
#include "auv_inspection_planner/adapters/maps.hpp"
struct QuietDiagnostics final : inspection::Diagnostics {
  void text(const std::string&,bool,bool) override {}
  void info(const std::string&) override {}
  void warning(const std::string&) override {}
  void error(const std::string&) override {}
};
#endif

int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const std::string prefix=argv[1];
  std::ofstream results(prefix+".txt");
  results<<std::setprecision(17);
#ifdef ORIGINAL_MAPPER
  MapperConfig config;
  config.info_gain_threshold=100.0;
  RealTimeStereoMapper mapping(config);
#else
  const inspection::MapperConfig config{90.,256.,.5,.1,4.,.5,-32.,100.,1.2,.02};
  QuietDiagnostics diagnostics;
  inspection::OctomapAdapter map(config.resolution,diagnostics);
  inspection::PclIndexes indexes;
  inspection::RoutePlanner routes(config.resolution,diagnostics);
  inspection::MapUpdate mapping(config,map,map);
  inspection::CandidatePlanner candidates(config,map,indexes,routes,diagnostics);
#endif
  for(int frame=0;frame<3;++frame) {
    cv::Mat depth(16,16,CV_32FC1), color(16,16,CV_8UC3);
    for(int v=0;v<16;++v) for(int u=0;u<16;++u) {
      float z=1.0f+.05f*u+.025f*v;
      if(u==0) z=0.5f;
      if(u==1) z=4.0f;
      if(u==2) z=std::numeric_limits<float>::quiet_NaN();
      depth.at<float>(v,u)=z;
      color.at<cv::Vec3b>(v,u)=cv::Vec3b(u+frame,v+frame,u+v);
    }
    const Eigen::Vector3d position(frame*.2,frame*.1,-20.);
#ifdef ORIGINAL_MAPPER
    mapping.process_frame(position,.1*frame,.02*frame,-.03*frame,color,depth);
    results<<mapping.get_current_leaves()<<'\n';
#else
    std::vector<float> pixels;
    std::vector<std::array<unsigned char,3>> colors;
    for(int v=0;v<16;++v) for(int u=0;u<16;++u) {
      pixels.push_back(depth.at<float>(v,u));
      const auto c=color.at<cv::Vec3b>(v,u); colors.push_back({c[0],c[1],c[2]});
    }
    mapping.process_frame(position,.1*frame,.02*frame,-.03*frame,
      inspection::ColorImage{16,16,std::move(colors)},inspection::DepthImage{16,16,std::move(pixels)});
    results<<map.count()<<'\n';
#endif
  }
#ifdef ORIGINAL_MAPPER
  const auto plan=mapping.generate_candidate_viewpoints(Eigen::Vector3d(0.,0.,-20.));
  mapping.save_map(prefix+".bt");
#else
  const auto plan=candidates.generate_candidate_viewpoints(Eigen::Vector3d(0.,0.,-20.));
  map.save(prefix+".bt");
#endif
  results<<plan.first.size()<<' '<<plan.second.size()<<'\n';
  for(const auto& p:plan.first) results<<p.transpose()<<'\n';
  for(const auto& p:plan.second) results<<p.transpose()<<'\n';
}
