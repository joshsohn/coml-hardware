/**
 * @file imu.cpp
 * @brief API for accessing IMU (via imu_app api)
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2021
 */

#include <iostream>
#include <stdexcept>
#include <string>

#include "snap_ipc/imu.h"

#include "ipc/common.h"

namespace acl {
namespace snapipc {

IMU::IMU(const std::string& vehname)
: vehname_(vehname)
{
  // unique key to access the same shmem location
  const size_t imukey = acl::ipc::createKeyFromStr(vehname, "imu");
  imuclient_.reset(new acl::ipc::Client<Data>(imukey));

  // start reading in separate thread
  thread_stop_ = false;
  read_thread_ = std::thread(&IMU::read_thread, this);
}

// ----------------------------------------------------------------------------

IMU::~IMU()
{
  thread_stop_ = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

// ----------------------------------------------------------------------------

void IMU::register_imu_cb(Callback cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_ = cb;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void IMU::read_thread()
{
  const int32_t returned_sample_count = 1; // only one recvd sample at a time
  uint32_t prev_sequence_number = 0;
  uint32_t current_seqeunce_number;

  std::vector<Data> data(1, Data());

  while (!thread_stop_) {    
    // blocking call which times out after 3 seconds
    bool success = imuclient_->read(&data[0]);

    if (!success) {
      std::cout << "[WARN] IMU: Error getting samples from api" << std::endl;
    } else {
      current_seqeunce_number = data[0].seq;
      if (prev_sequence_number != 0 && prev_sequence_number + 1 != current_seqeunce_number) {
        std::cout << "[WARN] IMU: Missed samples, expected " << (prev_sequence_number+1)
                  << ", got " << current_seqeunce_number << ". Sample count: "
                  << returned_sample_count << std::endl;
      }
      prev_sequence_number = current_seqeunce_number;

      { // call handler if exists
        std::lock_guard<std::mutex> lock(mtx_);
        if (cb_) cb_(data);
      }
    }
  }
  std::cout << "[INFO] IMU: Exiting read thread" << std::endl;
}

} // ns snapipc
} // ns acl
