#include "controlActive.h"
#include "RateTimer.h"
#include "sensor/sensor.h"
#include "thrid_party/glog/logging.h"
#include <thread>
using namespace event;
using namespace qp;
using namespace std;
using QState = qp::QHsm::QState;

std::shared_ptr<ControlActive> ControlActive::create(std::shared_ptr<qp::Qf> qf) {
  return std::make_shared<ControlActive>(qf);
}

void ControlActive::initial(std::shared_ptr<qp::QEvent> e) {
  LOG_INFO << " DockActive init";
  p2p_         = std::make_shared<control::P2P>();
  followall_   = std::make_shared<control::Followall>();
  lp2p_        = std::make_shared<control::LP2P>();
  np2p_        = std::make_shared<control::NP2P>();
  ap2p_        = std::make_shared<control::AP2P>();
  path_search_ = std::make_shared<nav::JpsSearch>();
  subscribe(Type::CONTROL_FOLLOWALL);
  subscribe(Type::CONTROL_LP2P);
  subscribe(Type::CONTROL_NP2P);
  subscribe(Type::CONTROL_FIND_PATH);
  subscribe(Type::CONTROL_AP2P);
  subscribe(Type::CONTROL_GO_BACK_FORWARD);
  subscribe(Type::CONTROL_TROUBLE_RECOVERY);
  Q_INIT(&ControlActive::running);
}

qp::QSTATE ControlActive::running(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  running";
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit running";
      return nullptr;
    }
    case Type::CONTROL_FOLLOWALL: {
      LOG_INFO << "CONTROL_FOLLOWALL";
      Q_TRAN(&ControlActive::followWallControl);
      return nullptr;
    }
    case Type::CONTROL_LP2P: {
      LOG_INFO << "CONTROL_FOLLOWALL";
      auto req = static_pointer_cast<SyncEventWithData<common::Lp2pData>>(e);
      lp2p_->setGoalPose(req->data.goal_pose);
      lp2p_->setNearGoalDistance(req->data.distance);
      lp2p_->setId(req->id);
      Q_TRAN(&ControlActive::lp2pControl);
      return nullptr;
    }
    case Type::CONTROL_GO_BACK_FORWARD: {
      auto req      = static_pointer_cast<SyncEventWithData<common::GoBackData>>(e);
      go_back_data_ = req->data;
      p2p_->setId(req->id);
      Q_TRAN(&ControlActive::goBackForwardControl);
      return nullptr;
    }
    case Type::CONTROL_NP2P: {
      auto req = static_pointer_cast<SyncEventWithData<common::Np2pData>>(e);
      np2p_->setData(req->data);
      np2p_->setId(req->id);
      Sensor::GetInstance()->publishPath("nav_path", req->data.path);
      Q_TRAN(&ControlActive::np2pControl);
      return nullptr;
    }
    case Type::CONTROL_AP2P: {
      LOG_INFO << "CONTROL_AP2P";
      auto req = static_pointer_cast<SyncEventWithData<common::Ap2pData>>(e);
      ap2p_->setAP2PGoal(req->data.goal_pose, req->data.next_pose);
      ap2p_->setId(req->id);
      Q_TRAN(&ControlActive::ap2pControl);
      return nullptr;
    }
    case Type::CONTROL_FIND_PATH: {
      LOG_INFO << "CONTROL_FIND_PATH";
      auto req        = static_pointer_cast<SyncEventWithData<common::FindPathData>>(e);
      find_path_data_ = req->data;
      path_search_->setId(req->id);
      Q_TRAN(&ControlActive::findPath);
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::top;
    }
  }
}

qp::QSTATE ControlActive::followWallControl(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  followWallControl";
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit followWallControl";
      common::Twist twist;
      Sensor::GetInstance()->publishCmdVel(twist);
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose      robot_pose = Sensor::GetInstance()->getSlamPose();
      common::LaserScan laser_scan = Sensor::GetInstance()->getLaserScan();
      common::Bumper    bumper     = Sensor::GetInstance()->getBumper();
      common::Twist     twist      = followall_->getFollowallTwsit(robot_pose, laser_scan, bumper);
      Sensor::GetInstance()->publishCmdVel(twist);
      rateTimer.sleep();
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}

qp::QSTATE ControlActive::goBackForwardControl(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  goBackForwardControl";
      common::Pose robot_pose = Sensor::GetInstance()->getSlamPose();
      p2p_->setStartPose(robot_pose);
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit goBackForwardControl";
      common::Twist twist;
      Sensor::GetInstance()->publishCmdVel(twist);
      return nullptr;
    }
    case Type ::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Twist  twist;
      common::Bumper bumper;
      bumper = Sensor::GetInstance()->getBumper();
      if (go_back_data_.use_distance) {
        common::Pose robot_pose = Sensor::GetInstance()->getSlamPose();
        if (p2p_->goDistance(robot_pose, go_back_data_.distance, (common::ControlDir)go_back_data_.go_type, twist)) {
          publish(make_shared<SyncEventWithData<int>>(p2p_->getId(), Type::CONTROL_GO_BACK_FORWARD_FINISH,
                                                      common::ControlFinishResult::GO_BACK_FRONT_SUCCESS));
          LOG_INFO << "go back front success";
          Q_TRAN(&ControlActive::running);
          return nullptr;
        }
      } else {
        if (p2p_->goTime(go_back_data_.time, (common::ControlDir)go_back_data_.go_type, twist)) {
          publish(make_shared<SyncEventWithData<int>>(p2p_->getId(), Type::CONTROL_GO_BACK_FORWARD_FINISH,
                                                      common::ControlFinishResult::GO_BACK_FRONT_SUCCESS));
          LOG_INFO << "go back front success";
          Q_TRAN(&ControlActive::running);
          return nullptr;
        }
      }
      if (bumper.right_bumper || bumper.left_bumper) {
        publish(make_shared<SyncEventWithData<int>>(p2p_->getId(), Type::CONTROL_GO_BACK_FORWARD_FINISH,
                                                    common::ControlFinishResult::GO_BACK_FRONT_FAIL));
        LOG_INFO << "go back front fail";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      Sensor::GetInstance()->publishCmdVel(twist);
      rateTimer.sleep();
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}

qp::QSTATE ControlActive::lp2pControl(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  lp2pControl";
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit lp2pControl";
      common::Twist twist;
      Sensor::GetInstance()->publishCmdVel(twist);
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose      robot_pose = Sensor::GetInstance()->getSlamPose();
      common::LaserScan laser_scan = Sensor::GetInstance()->getLaserScan();
      common::Bumper    bumper     = Sensor::GetInstance()->getBumper();
      common::Twist     twist;
      if (lp2p_->getLp2pTwsit(robot_pose, laser_scan, bumper, twist) == true) {
        publish(make_shared<SyncEventWithData<int>>(lp2p_->getId(), Type::CONTROL_LP2P_FINISH,
                                                    common::ControlFinishResult::LP2P_SUCCESS));
        LOG_INFO << "lp2p success";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      if (lp2p_->getLp2pStatus() != common::ControlStatus::CONTROL_TURN
          && (bumper.right_bumper || bumper.left_bumper)) {
        publish(make_shared<SyncEventWithData<int>>(lp2p_->getId(), Type::CONTROL_LP2P_FINISH,
                                                    common::ControlFinishResult::LP2P_FAIL));
        LOG_INFO << "lp2p fail";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      Sensor::GetInstance()->publishCmdVel(twist);
      rateTimer.sleep();
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}


qp::QSTATE ControlActive::ap2pControl(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  ap2pControl";
      ap2p_->reset();
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit ap2pControl";
      common::Twist twist;
      Sensor::GetInstance()->publishCmdVel(twist);
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose   robot_pose = Sensor::GetInstance()->getSlamPose();
      common::Bumper bumper     = Sensor::GetInstance()->getBumper();
      common::Twist  twist;
      if (ap2p_->getAp2pTwist(robot_pose, twist) == true) {
        publish(make_shared<SyncEventWithData<int>>(ap2p_->getId(), Type::CONTROL_AP2P_FINISH,
                                                    common::ControlFinishResult::AP2P_SUCCESS));
        LOG_INFO << "ap2p success";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      if (bumper.left_bumper == true || bumper.right_bumper == true) {
        publish(make_shared<SyncEventWithData<int>>(ap2p_->getId(), Type::CONTROL_AP2P_FINISH,
                                                    common::ControlFinishResult::AP2P_FAIL));
        LOG_INFO << "ap2p fail";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      Sensor::GetInstance()->publishCmdVel(twist);
      rateTimer.sleep();
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}

qp::QSTATE ControlActive::np2pControl(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  np2pControl";
      np2p_->reset();
      pushBack(make_shared<Event>(Type::STEP));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit np2pControl";
      common::Twist twist;
      Sensor::GetInstance()->publishCmdVel(twist);
      return nullptr;
    }
    case Type::STEP: {
      pushBack(make_shared<Event>(Type::STEP));
      common::Pose   robot_pose = Sensor::GetInstance()->getSlamPose();
      common::Twist  twist;
      common::Bumper bumper = Sensor::GetInstance()->getBumper();
      if (np2p_->getNp2pTwist(robot_pose, twist)) {
        publish(make_shared<SyncEventWithData<int>>(np2p_->getId(), Type::CONTROL_NP2P_FINISH,
                                                    common::ControlFinishResult::NP2P_SUCCESS));
        LOG_INFO << "np2p success";
        Q_TRAN(&ControlActive::running);
        return nullptr;
      }
      Sensor::GetInstance()->publishCmdVel(twist);
      rateTimer.sleep();
      return nullptr;
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}

qp::QSTATE ControlActive::findPath(std::shared_ptr<qp::QEvent> e) {
  switch (e->sig) {
    case Type::ENTRY: {
      LOG_INFO << "enter  findPath";
      auto map = Sensor::GetInstance()->getMapData();
      Sensor::GetInstance()->updateCostmap(map);
      auto costmap_ptr = Sensor::GetInstance()->getCostmap2d();
      path_search_->setMap(std::make_shared<costmap_2d::Costmap2D>(costmap_ptr));
      pushBack(make_shared<Event>(Type::CONTROL_START_FIND_PATH));
      return nullptr;
    }
    case Type::EXIT: {
      LOG_INFO << "exit findPath";
      return nullptr;
    }
    case Type::STEP: {
      return nullptr;
    }
    case Type::CONTROL_START_FIND_PATH: {
      LOG_INFO << "CONTROL_START_FIND_PATH";
      pushBack(make_shared<Event>(Type::STEP));
      std::vector<common::Pose> path;
      path_search_->findPath(find_path_data_.start_pose, find_path_data_.goal_pose, path,
                             find_path_data_.find_replace_goal);
      publish(make_shared<SyncEventWithData<std::vector<common::Pose>>>(path_search_->getId(),
                                                                        Type::CONTROL_FIND_PATH_FINSIH, path));
      Q_TRAN(&ControlActive::running);
    }
    default: {
      return (QSTATE)&ControlActive::running;
    }
  }
}

