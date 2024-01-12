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

#include "SnapdragonEscManager.hpp"
#include "SnapdragonDebugPrint.h"

namespace Snapdragon {

EscManager::EscManager(const std::string& vehname)
: vehname_(vehname), armed_(false)
{
#ifdef SNAP_SIM
    client_.reset(new acl::snapipc::Client(vehname));
#else
    client_.reset(new acl::snapipc::Client());
#endif

    // send a valid 'off' signal
    disarm();
}

// ----------------------------------------------------------------------------

EscManager::~EscManager()
{
    // turn off the motors on the way out
    disarm();
}

// ----------------------------------------------------------------------------

int32_t EscManager::Initialize()
{
    // bool success = escs->init();
    // if (!success) return -1;

    disarm();

    return 0;
}

// ----------------------------------------------------------------------------

int32_t EscManager::Terminate()
{
    // escs->close();
    disarm();
    return 0;
}

// ----------------------------------------------------------------------------

void EscManager::update(const std::array<float, 6>& throttle)
{
    if (!armed_) return;

    std::lock_guard<std::mutex> lock(mutex_);
    client_->set_motors(throttle);
}

// ----------------------------------------------------------------------------

bool EscManager::arm()
{
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = true;
    return true;
}

// ----------------------------------------------------------------------------

bool EscManager::disarm()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<float, 6> throttle;
    throttle.fill(0.0f);
    client_->set_motors(throttle);
    armed_ = false;
    return true;

}

// ----------------------------------------------------------------------------

bool EscManager::isArmed()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return armed_;
}

} // ns Snapdragon
