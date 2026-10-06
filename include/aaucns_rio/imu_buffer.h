// Copyright (C) 2024 Jan Michalczyk, Control of Networked Systems, University
// of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <jan.michalczyk@aau.at>

#ifndef _IMU_BUFFER_H_
#define _IMU_BUFFER_H_

#include <sensor_msgs/Imu.h>

#include <deque>
#include <mutex>

namespace aaucns_rio
{
// Thread-safe FIFO of IMU measurements received but not yet used for
// prediction. Filled from the IMU thread, emptied by whoever holds the filter.
class ImuBuffer
{
   public:
    void push(const sensor_msgs::ImuConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push_back(msg);
    }

    // Take all buffered measurements, oldest first.
    std::deque<sensor_msgs::ImuConstPtr> popAll()
    {
        std::deque<sensor_msgs::ImuConstPtr> measurements;
        std::lock_guard<std::mutex> lock(mutex_);
        measurements.swap(buffer_);
        return measurements;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }

   private:
    std::mutex mutex_;
    std::deque<sensor_msgs::ImuConstPtr> buffer_;
};

}  // namespace aaucns_rio

#endif /* _IMU_BUFFER_H_ */
