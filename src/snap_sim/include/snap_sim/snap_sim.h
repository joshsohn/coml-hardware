/**
 * @file snap_sim.h
 * @brief ROS wrapper for ACL multirotor dynamic simulation
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 23 December 2019
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>

#include "snap_sim/physics_engine.h"
#include "snap_sim/multirotor.h"

#include "ipc/client.h"
#include "ipc/server.h"
#include "ipc/common.h"
#include "snap_ipc/imu.h"
#include "snap_ipc/client.h"

namespace acl {
namespace snap_sim {

  class SnapSim
  {
  public:
    SnapSim(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~SnapSim();
    
  private:
    ros::NodeHandle nh_, nhp_;
    ros::Timer tim_imu_, tim_mocap_;
    ros::Publisher pub_pose_, pub_twist_, pub_vizmesh_;
    tf2_ros::TransformBroadcaster br_;

    /// \brief Parameters
    std::string name_; ///< name of the simulated vehicle
    std::string mocap_parent_frame_; ///< pose w.r.t what?
    bool mocap_tf_broadcast_; ///< should we broadcast UAV pose on tf tree?
    bool viz_mesh_; ///< should we publish a mesh marker for rviz
    std::string mesh_uri_; ///< path of mesh
    double mesh_scale_; ///< scaling on mesh visualization
    double imu_dt_; ///< the simulation is driven by IMU measurements
    double mocap_dt_; ///< period at which to broadcast simulated mocap

    /// \brief Internal state
    snapipc::IMU::Data imu_; ///< imu data message
    Multirotor::MotorCmds motorcmds_; ///< throttle commands in [0,1]

    /// \brief Components
    PhysicsEngine physics_; ///< physics world to simulate rigid bodies
    MultirotorPtr multirotor_; ///< multirotor geometry, dynamics, and actuator
    std::unique_ptr<ipc::Server<snapipc::IMU::Data>> imuserver_;
    std::unique_ptr<ipc::Client<throttle_commands>> escclient_;

    /// \brief Thread for reading esc data
    std::thread escthread_;
    std::mutex escmtx_;

    /// \brief Thread for auto arming snap stack
    std::thread armthread_;

    /// \brief ros::Time of the last call to the simStepCb function
    ros::Time time_last_simStepCb_;

    void init();
    void armThread();
    void escReadThread();
    void simStepCb(const ros::TimerEvent& e);
    void mocapCb(const ros::TimerEvent& e);
  };

} // ns snap_sim
} // ns acl
