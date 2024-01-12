/**
 * @file imu.h
 * @brief API for accessing IMU (via imu_app api)
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 April 2021
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ipc/client.h"

namespace acl {
namespace snapipc {

  class IMU
  {
  public:
    /**
     * @brief      IMU data returned via callback
     */
    struct Data
    {
      uint64_t usec; ///< timestamp in seconds
      uint32_t seq; ///< sequence number
      float acc_x, acc_y, acc_z; ///< units of [g/s^2]
      float gyr_x, gyr_y, gyr_z; ///< units of [rad/s]
    };

    // signature to use when registering a callback
    using Callback = std::function<void(const std::vector<Data>& samples)>;

  public:
    IMU(const std::string& vehname);
    ~IMU();

    /**
     * @brief      Register a callback that is fired when
     *             batches of IMU data are available.
     *
     * @param[in]  cb    Callback conforming to Callback signature.
     */
    void register_imu_cb(Callback cb);

  private:
    const std::string vehname_;

    // IPC client to receive IMU samples with
    std::unique_ptr<acl::ipc::Client<Data>> imuclient_;

    //\brief Threading stuff
    std::atomic<bool> thread_stop_;
    std::mutex mtx_;
    std::thread read_thread_;

    Callback cb_; ///< user's callback

    /**
     * @brief      IMU reader, called in a separate thread
     */
    void read_thread();

  };

} // ns snapipc
} // ns acl
