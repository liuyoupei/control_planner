#include "cleanActive.h"
#include "common/publish_node.h"
#include "common/sub_node.h"
#include "controlActive.h"
#include "qassert.h"
#include "qf.h"
#include "thrid_party/glog/logging.h"
#include "virtualObstanceActive.h"
#include <assert.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
DEFINE_string(config_path, "/home/lyp/philips_clean_robot/src/control_planner/config/config.yaml", "yaml config path");
using namespace qp;
using namespace std;
using QState = qp::QHsm::QState;
void onAssert__(char const* file, int line) {
  fprintf(stderr, "Assertion failed in %s, line %d", file, line);
  assert(0);
  exit(-1);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "clean_robot");
  RosSubscribeNode sub_node;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_colorlogtostderr          = true;
  FLAGS_max_log_size              = 1024;
  FLAGS_stop_logging_if_full_disk = true;
  FLAGS_logbufsecs                = 0;
  YAML::Node  config              = YAML::LoadFile(FLAGS_config_path);
  std::string log_path            = config["log_path"].as<std::string>();

  google::SetLogDestination(google::GLOG_FATAL, log_path.c_str());
  google::SetLogDestination(google::GLOG_ERROR, log_path.c_str());
  google::SetLogDestination(google::GLOG_WARNING, log_path.c_str());
  google::SetLogDestination(google::GLOG_INFO, log_path.c_str());
  auto qf = make_shared<qp::Qf>();
  qf->registerActor(CleanActive::create(qf));
  qf->registerActor(ControlActive::create(qf));
  qf->registerActor(VirtualObstanceActive::create(qf));
  qf->start();
  return 0;
}
