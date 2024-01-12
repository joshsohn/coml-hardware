/****************************************************************************
 *   Copyright (c) 2017 Brett T. Lopez. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name snap nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <chrono>
#include <functional>

#include "SnapdragonObserverManager.hpp"
#include "SnapdragonUtils.hpp"
#include "SnapdragonDebugPrint.h"

Snapdragon::ObserverManager::ObserverManager()
{
  smc_man_ptr_ = nullptr;
  esc_man_ptr_ = nullptr;
  initialized_ = false;
  calibrated_ = false;
  got_pose_ = false;
  time_filter_initialized_ = false;
  if_time_filter_ = false;
}

Snapdragon::ObserverManager::~ObserverManager()
{
  CleanUp();
}

int32_t Snapdragon::ObserverManager::CleanUp()
{
  // stop the imu manager
  if (imu_ != nullptr) imu_.reset();

  // stop the attitude control manager
  if (smc_man_ptr_ != nullptr)
  {
    delete smc_man_ptr_;
    smc_man_ptr_ = nullptr;
  }
  // stop the esc manager
  if (esc_man_ptr_ != nullptr)
  {
    esc_man_ptr_->Terminate();
    delete esc_man_ptr_;
    esc_man_ptr_ = nullptr;
  }

  return 0;
}

int32_t Snapdragon::ObserverManager::Initialize(const std::string& vehname,
                                                const Snapdragon::ObserverManager::InitParams& observer_params,
                                                const Snapdragon::ControllerManager::InitParams& smcparams)
{
  state_ = State();
  imu_data_ = Data();

  observer_params_ = observer_params;
  int32_t rc = 0;

  if (rc == 0)
  {
    // initialize imu
#ifdef SNAP_SIM
    imu_.reset(new acl::snapipc::IMU(vehname));
#else
    imu_.reset(new acl::snapipc::IMU("imu0"));
#endif

    // initialize the Controller Manager
    smc_man_ptr_ = new Snapdragon::ControllerManager();
    smc_man_ptr_->Initialize(smcparams);

    // initialize the Comm Manager
    esc_man_ptr_ = new Snapdragon::EscManager(vehname);
    rc = esc_man_ptr_->Initialize();
  }

  if (rc != 0)
  {
    ERROR_PRINT("Error initializing the Ukf Manager.");
    CleanUp();
  }
  else
  {
    initialized_ = true;
  }
  return 0;
}

int32_t Snapdragon::ObserverManager::Start()
{
  using namespace std::placeholders;
  int32_t rc = 0;
  if (initialized_)
  {
    // connect the imu callback
    imu_->register_imu_cb(std::bind(&ObserverManager::imu_cb, this, _1));
  }
  else
  {
    ERROR_PRINT("Calling Start without calling intialize");
    rc = -1;
  }
  return rc;
}

int32_t Snapdragon::ObserverManager::Stop()
{
  CleanUp();
  return 0;
}

void Snapdragon::ObserverManager::imu_cb(const std::vector<acl::snapipc::IMU::Data>& imu_samples)
{
  static constexpr size_t CALIB_SAMPLES = 1000;
  static double Ts = 0; // estimated sample period [sec]

  static std::chrono::time_point<std::chrono::high_resolution_clock> t1 = std::chrono::high_resolution_clock::now();
  const auto t2 = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration<double>(t2 - t1);
  imu_data_.loop_time = duration.count();
  t1 = std::chrono::high_resolution_clock::now();

  for (int ii = 0; ii < imu_samples.size(); ++ii)
  {
    uint64_t current_timestamp_ns = imu_samples[ii].usec * 1000;

    // make sure we didn't skip too big of a gap in IMU measurements
    static uint64_t last_timestamp = current_timestamp_ns;
    float delta = (current_timestamp_ns - last_timestamp) * 1e-6;
    static constexpr float imu_sample_dt_reasonable_threshold_ms = 12.5;
    if (delta > imu_sample_dt_reasonable_threshold_ms)
    {
      WARN_PRINT("IMU sample dt > %f ms -- %f ms", imu_sample_dt_reasonable_threshold_ms, delta);
    }
    last_timestamp = current_timestamp_ns;

    float lin_acc[3], ang_vel[3];

    // Convert from imu (frd) to body (flu)
    lin_acc[0] =  imu_samples[ii].acc_x;
    lin_acc[1] = -imu_samples[ii].acc_y;
    lin_acc[2] = -imu_samples[ii].acc_z;
    ang_vel[0] =  imu_samples[ii].gyr_x;
    ang_vel[1] = -imu_samples[ii].gyr_y;
    ang_vel[2] = -imu_samples[ii].gyr_z;

    static uint32_t sequence_number_last = 0;
    int num_dropped_samples = 0;
    if (sequence_number_last != 0)
    {
      // The diff should be 1, anything greater means we dropped samples
      num_dropped_samples = imu_samples[ii].seq - sequence_number_last - 1;
      if (num_dropped_samples > 0)
      {
        WARN_PRINT("Current IMU sample = %u, last IMU sample = %u", imu_samples[ii].seq,
                   sequence_number_last);
      }
    }
    sequence_number_last = imu_samples[ii].seq;

    static int count = 0;

    if (!calibrated_) {
      static constexpr float kNormG = 9.80665f;

      // Update accel and gyro bias initialization
      count++;
      state_.accel_bias.x += lin_acc[0];
      state_.accel_bias.y += lin_acc[1];
      state_.accel_bias.z += (lin_acc[2] - kNormG);
      state_.gyro_bias.x += ang_vel[0];
      state_.gyro_bias.y += ang_vel[1];
      state_.gyro_bias.z += ang_vel[2];

      // accumulate time deltas (in seconds)
      Ts += delta * 1e-3;

      if (count == CALIB_SAMPLES) {
        // Average accel and gyro biases
        state_.accel_bias.x /= CALIB_SAMPLES;
        state_.accel_bias.y /= CALIB_SAMPLES;
        state_.accel_bias.z /= CALIB_SAMPLES;
        state_.gyro_bias.x /= CALIB_SAMPLES;
        state_.gyro_bias.y /= CALIB_SAMPLES;
        state_.gyro_bias.z /= CALIB_SAMPLES;

        // capture estimated sample period
        Ts /= CALIB_SAMPLES;

        // compute LPF alpha parameter
        alpha_accel_xy_ = computeLPFGain(observer_params_.fc_acc_xy, Ts);
        alpha_accel_z_ = computeLPFGain(observer_params_.fc_acc_z, Ts);
        alpha_gyro_ = computeLPFGain(observer_params_.fc_gyr, Ts);

        // initialize adaptive notches
        if (observer_params_.anotch_enable) {
          observer_params_.anotch_params.Fs = 1./Ts;
          for (size_t i=0; i<anotch_gyr_.size(); i++) {
            anotch_gyr_[i].reset(new adaptnotch::AdaptiveNotch(observer_params_.anotch_params));
          }
        }

        calibrated_ = true;
        WARN_PRINT("Calibration complete");
      }
    } else {
      // Apply accel and gyro biases to raw IMU measurements
      float temp_acc[3], temp_gyro[3];
      temp_acc[0]  = (lin_acc[0]-state_.accel_bias.x);
      temp_acc[1]  = (lin_acc[1]-state_.accel_bias.y);
      temp_acc[2]  = (lin_acc[2]-state_.accel_bias.z);
      temp_gyro[0] = (ang_vel[0]-state_.gyro_bias.x);
      temp_gyro[1] = (ang_vel[1]-state_.gyro_bias.y);
      temp_gyro[2] = (ang_vel[2]-state_.gyro_bias.z);

      // filter newly sampled raw IMU data
      filter(imu_data_, temp_acc, temp_gyro);

      imu_data_.sequence_number = sequence_number_last;
      imu_data_.current_timestamp_ns = current_timestamp_ns;

      state_.sequence_number = sequence_number_last;
      state_.current_timestamp_ns = current_timestamp_ns;

      // Propagate state
      propagateState(state_, imu_data_, delta * 1e-3);
    }
  }

  if (calibrated_ && got_pose_) {
    // Update attitude state
    smc_man_ptr_->updateAttState(smc_man_ptr_->smc_state_, state_.q, state_.w);
    
    static double last_timestamp_ns = 0.0;
    const double dt=(state_.current_timestamp_ns - last_timestamp_ns)/1e9; //In seconds
    last_timestamp_ns=state_.current_timestamp_ns;
    // Update Motor commands
    smc_man_ptr_->updateMotorCommands(dt, smc_man_ptr_->throttles_, smc_man_ptr_->smc_des_, smc_man_ptr_->smc_state_);

    // write out to PWM ESCs
    esc_man_ptr_->update(smc_man_ptr_->throttles_);

    // Copy new motor commands to kf public variable
    smc_motors_ = smc_man_ptr_->throttles_;
    smc_data_ = smc_man_ptr_->smc_data_;
  }
}

int32_t Snapdragon::ObserverManager::propagateState(Snapdragon::ObserverManager::State& state,
                                                    Snapdragon::ObserverManager::Data data, float dt)
{
  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock(sync_mutex_);

  Quaternion q = state.q;
  Vector world_lin_acc;

  // Transform accel from body to world frame
  world_lin_acc.x = (1 - 2 * (q.y * q.y + q.z * q.z)) * data.lin_accel[0] +
                    2 * (q.y * q.x - q.z * q.w) * data.lin_accel[1] + 2 * (q.x * q.z + q.y * q.w) * data.lin_accel[2];
  world_lin_acc.y = (1 - 2 * (q.x * q.x + q.z * q.z)) * data.lin_accel[1] +
                    2 * (q.y * q.z - q.x * q.w) * data.lin_accel[2] + 2 * (q.x * q.y + q.z * q.w) * data.lin_accel[0];
  world_lin_acc.z = (1 - 2 * (q.x * q.x + q.y * q.y)) * data.lin_accel[2] +
                    2 * (q.x * q.z - q.y * q.w) * data.lin_accel[0] + 2 * (q.y * q.z + q.x * q.w) * data.lin_accel[1];

  // Accel propogation
  state.pos.x += state.vel.x * dt + 0.5 * dt * dt * world_lin_acc.x;
  state.pos.y += state.vel.y * dt + 0.5 * dt * dt * world_lin_acc.y;
  state.pos.z += state.vel.z * dt + 0.5 * dt * dt * (world_lin_acc.z - 9.80665f);

  state.vel.x += world_lin_acc.x * dt;
  state.vel.y += world_lin_acc.y * dt;
  state.vel.z += (world_lin_acc.z - 9.80665f) * dt;

  // Gyro propogation
  state.q.w -= 0.5 * (q.x * data.ang_vel[0] + q.y * data.ang_vel[1] + q.z * data.ang_vel[2]) * dt;
  state.q.x += 0.5 * (q.w * data.ang_vel[0] - q.z * data.ang_vel[1] + q.y * data.ang_vel[2]) * dt;
  state.q.y += 0.5 * (q.z * data.ang_vel[0] + q.w * data.ang_vel[1] - q.x * data.ang_vel[2]) * dt;
  state.q.z += 0.5 * (q.x * data.ang_vel[1] - q.y * data.ang_vel[0] + q.w * data.ang_vel[2]) * dt;

  // Ensure quaterion is properly normalized
  float norm = sqrt(state.q.w * state.q.w + state.q.x * state.q.x + state.q.y * state.q.y + state.q.z * state.q.z);
  state.q.w /= norm;
  state.q.x /= norm;
  state.q.y /= norm;
  state.q.z /= norm;

  state.w.x = data.ang_vel[0];
  state.w.y = data.ang_vel[1];
  state.w.z = data.ang_vel[2];

  return 0;
}

int32_t Snapdragon::ObserverManager::updateState(Snapdragon::ObserverManager::State& state, Vector pos, Quaternion q,
                                                 uint64_t timestamp_us, uint64_t ros_timestamp_us, uint32_t current_seq)
{
  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock(sync_mutex_);

  if (!got_pose_)
  {
    // on the first external pose message, just override the filter pose
    // with the external pose measurement.
    state.updateState(pos, q);
    got_pose_ = true;
  } 
  else {

    float dt = (timestamp_us - last_pose_update_us_) * 1e-6; //timestamps_us and last_pose_update_us_ are both in microseconds and dt is in seconds
    float ros_dt = (ros_timestamp_us - last_ros_update_us_) * 1e-6;
    bool skipped = false;
    static float exp_dt = 0;

    // Time filter
    if (if_time_filter_){

      // TIME FILTER: after communication outage, we tend to receive clumped state data and need to get rid of them
      // TODO: we may wanna keep the last data in the clump but right now we all toss away all the clumped data

      static constexpr size_t TIME_FILTER_CALIB_SAMPLES = 1000; // how many data we use to calibrate the time filter
      static int time_filter_count = 0;
      
      if (!time_filter_initialized_) { // if the time filter is not initialized, then find the average dt to find an expected ros_dt

        if (time_filter_count < TIME_FILTER_CALIB_SAMPLES){ // still calibrating
          exp_dt += dt;
        }
        else {// once we have enough samples
          exp_dt /= TIME_FILTER_CALIB_SAMPLES;
          WARN_PRINT("exp_dt is found: exp_dt = %f seconds",exp_dt);
          time_filter_initialized_ = true;
          WARN_PRINT("Time filter calibration complete");
        }

        time_filter_count++;

      } else {// once the time filter is initialed, snap starts throwing away clumped data

        // no longer used since we initialize exp_dt in the above if-statement 
        // float exp_dt = 10*1e-3; //in case of 100Hz
        // flatt exp_dt = 5*1e-3; //in case of 200Hz
      
        if (ros_dt > upper_bound_ * exp_dt || ros_dt < lower_bound_ * exp_dt)
        { //the heart of time filter
          // std::cout << "comm outage " << std::endl;
          // std::cout << "we throw away seq num " << current_seq << std::endl;
          // std::cout << "dt is " << dt << std::endl; 
          // std::cout << "ros_dt is " << ros_dt << std::endl; 
          last_pose_update_us_ = timestamp_us;
          last_ros_update_us_ = ros_timestamp_us;
          skipped = true;
        }
      }
    } 

    // update time filter related data
    time_filter_data_.dt = dt;
    time_filter_data_.ros_dt = ros_dt;
    time_filter_data_.skipped = skipped;
    time_filter_data_.upper = upper_bound_ * exp_dt;
    time_filter_data_.lower = lower_bound_ * exp_dt;

    if (skipped) return -1;

    if (dt <= 1e-3) return -1;

    Quaternion qe, qa, qu;
    qa.w = state.q.w;
    qa.x = state.q.x;
    qa.y = state.q.y;
    qa.z = state.q.z;
    qu.w = q.w;
    qu.x = q.x;
    qu.y = q.y;
    qu.z = q.z;

    // Generate error quaternion
    qConjProd(qe, qa, qu);

    // Update gyro bias
    state.gyro_bias.x -= observer_params_.Kgb * qe.x;
    state.gyro_bias.y -= observer_params_.Kgb * qe.y;
    state.gyro_bias.z -= observer_params_.Kgb * qe.z;

    Vector err;
    err.x = pos.x - state.pos.x;
    err.y = pos.y - state.pos.y;
    err.z = pos.z - state.pos.z;
    Vector berr;

    // TODO: should this be qa or q?
    berr.x = (1 - 2 * (qa.y * qa.y + qa.z * qa.z)) * err.x + 2 * (qa.y * qa.x + qa.z * qa.w) * err.y +
             2 * (qa.x * qa.z - qa.y * qa.w) * err.z;
    berr.y = (1 - 2 * (qa.x * qa.x + qa.z * qa.z)) * err.y + 2 * (qa.y * qa.z + qa.x * qa.w) * err.z +
             2 * (qa.x * qa.y - qa.z * qa.w) * err.x;
    berr.z = (1 - 2 * (qa.x * qa.x + qa.y * qa.y)) * err.z + 2 * (qa.x * qa.z + qa.y * qa.w) * err.x +
             2 * (qa.y * qa.z - qa.x * qa.w) * err.y;

    // Update accel bias
    state.accel_bias.x -= observer_params_.Kab * (berr.x);
    state.accel_bias.y -= observer_params_.Kab * (berr.y);
    state.accel_bias.z -= observer_params_.Kab * (berr.z);

    // Update state
    state.pos.x += observer_params_.Kp * err.x;
    state.pos.y += observer_params_.Kp * err.y;
    state.pos.z += observer_params_.Kp * err.z;

    state.vel.x += observer_params_.Kv * err.x / dt;
    state.vel.y += observer_params_.Kv * err.y / dt;
    state.vel.z += observer_params_.Kv * err.z / dt;

    state.q.w -= observer_params_.Kq * (state.q.w - q.w);
    state.q.x -= observer_params_.Kq * (state.q.x - q.x);
    state.q.y -= observer_params_.Kq * (state.q.y - q.y);
    state.q.z -= observer_params_.Kq * (state.q.z - q.z);
  }

  last_pose_update_us_ = timestamp_us;
  last_ros_update_us_ = ros_timestamp_us;
  return 0;
}

int32_t Snapdragon::ObserverManager::updateSMCState(desiredAttState newDesState)
{
  // Update desired attitude
  smc_man_ptr_->updateDesiredAttState(smc_man_ptr_->smc_des_, newDesState);

  return 0;
}

void Snapdragon::ObserverManager::filter(Data& imu, const float accel[3], const float gyro[3])
{
  // RC LPF accelerometer
  imu.lin_accel[0] = (1 - alpha_accel_xy_) * imu.lin_accel[0]  +  alpha_accel_xy_ * accel[0];
  imu.lin_accel[1] = (1 - alpha_accel_xy_) * imu.lin_accel[1]  +  alpha_accel_xy_ * accel[1];
  imu.lin_accel[2] = (1 - alpha_accel_z_)  * imu.lin_accel[2]  +  alpha_accel_z_  * accel[2];

  // RC LPF gyro
  imu.ang_vel[0] = (1 - alpha_gyro_) * imu.ang_vel[0]  +  alpha_gyro_ * gyro[0];
  imu.ang_vel[1] = (1 - alpha_gyro_) * imu.ang_vel[1]  +  alpha_gyro_ * gyro[1];
  imu.ang_vel[2] = (1 - alpha_gyro_) * imu.ang_vel[2]  +  alpha_gyro_ * gyro[2];

  // gyro adaptive notch filtering
  if (observer_params_.anotch_enable) {
    for (size_t i=0; i<3; i++) {
      imu.ang_vel[i] = anotch_gyr_[i]->apply(imu.ang_vel[i]);
    }
    // put the estimated peak frequency in a place it doesn't exactly belong...
    smc_man_ptr_->smc_data_.s.x = anotch_gyr_[0]->peakFreq();
    smc_man_ptr_->smc_data_.s.y = anotch_gyr_[1]->peakFreq();
    smc_man_ptr_->smc_data_.s.z = anotch_gyr_[2]->peakFreq();
  }

}

void Snapdragon::ObserverManager::initializeTimeFilter(bool time_filter, float upper_bound, float lower_bound){ // initialize if_time_filter_
  if_time_filter_ = time_filter;
  upper_bound_ = upper_bound;
  lower_bound_ = lower_bound;
}
