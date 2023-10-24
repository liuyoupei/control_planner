#include "virtualObstanceActive.h"
#include "RateTimer.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <thread>
using namespace event;
using namespace qp;
using namespace std;
using QState = qp::QHsm::QState;

std::shared_ptr<VirtualObstanceActive> VirtualObstanceActive::create(std::shared_ptr<qp::Qf> qf) {
  return std::make_shared<VirtualObstanceActive>(qf);
}

void VirtualObstanceActive::initial(std::shared_ptr<qp::QEvent> e) {
  LOG_INFO << " VirtualObstanceActive init";
  subscribe(Type::VIRTUAL_ROOM_LINE);
  subscribe(Type::VIRTUAL_START_CHECK_LINE);
  subscribe(Type::VIRTUAL_STOP_CHECK_LINE);
  subscribe(Type::CONTROL_FOLLOWALL);
  subscribe(Type::VIRTUAL_RESET);
  Q_INIT(&VirtualObstanceActive::running);
}

qp::QSTATE VirtualObstanceActive::running(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  running";
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit running";
      return nullptr;
    }
    case Type::STEP: {
      return nullptr;
    }
    case Type::VIRTUAL_ROOM_LINE: {
      LOG_INFO << "VIRTUAL_ROOM_LINE";
      auto req = static_pointer_cast<EvenData<std::vector<common::Edge>>>(e);
      line_check_.setRoomEdgeData(req->data);
      return nullptr;
    }
    case Type::VIRTUAL_START_CHECK_LINE: {
      LOG_INFO << "VIRTUAL_START_CHECK_LINE";
      Q_TRAN((qp::QSTATE)&VirtualObstanceActive::checkLine);
      line_check_.resetTrapping();
      return nullptr;
    }
    case Type::VIRTUAL_STOP_CHECK_LINE: {
      LOG_INFO << "VIRTUAL_STOP_CHECK_LINE";
      Q_TRAN((qp::QSTATE)&VirtualObstanceActive::running);
      return nullptr;
    }
    case Type::CONTROL_FOLLOWALL: {
      LOG_INFO << "CONTROL_FOLLOWALL";
      line_check_.setFollowallMark(Sensor::GetInstance()->getSlamPose());
      return nullptr;
    }
    case Type::VIRTUAL_RESET: {
      LOG_INFO << "VIRTUAL_RESET";
      line_check_.reset();
      return nullptr;
    }
    case Type::CONTROL_LP2P_FINISH: {
      auto resp = static_pointer_cast<SyncEventWithData<int>>(e);
      if (resp->data == common::ControlFinishResult::LP2P_SUCCESS) {
        LOG_INFO << "CONTROL_LP2P SUCCESS";
        line_check_.resetFollowall(Sensor::GetInstance()->getSlamPose());
      }
      return nullptr;
    }
    default: {
      return (QSTATE)&VirtualObstanceActive::top;
    }
  }
}

qp::QSTATE VirtualObstanceActive::checkLine(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  checkLine";
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit checkLine";
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose                           robot_pose = Sensor::GetInstance()->getSlamPose();
      common::Edge                           edge;
      std::vector<std::vector<common::Edge>> forbidden_zone;
      std::vector<common::Edge>              zone;
      if (line_check_.virtualLineCheck(robot_pose, edge)) {
        LOG_INFO << "find new virtual line edge_type " << edge.edge_type;
        LOG_INFO << "find new virtual line room_type " << edge.room_type;
        if (edge.room_type == common::RoomType::ROOM_DIVISION) {
          publish(make_shared<EvenData<common::Edge>>(Type::VIRTUAL_ROOM_LP2P_GOAL, edge));
        }
      }
      return nullptr;
    }
    default: {
      return (QSTATE)&VirtualObstanceActive::running;
    }
  }
}
