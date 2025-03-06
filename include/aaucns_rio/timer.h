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

#ifndef _TIMER_H_
#define _TIMER_H_

#include <chrono>
#include <exception>
#include <stdexcept>

namespace aaucns_rio
{
template <typename t>
t computeMovingAverage(const t& sample)
{
    static long int sample_number = 0;
    ++sample_number;
    static t sum = 0;
    sum = sum + sample;
    return (sum / sample_number);
}

class Timer
{
   public:
    Timer() : busy_(false) {}
    void start()
    {
        busy_ = true;
        start_ = std::chrono::high_resolution_clock::now();
    }
    double stop()
    {
        if (busy_)
        {
            reset();
            std::chrono::duration<double, std::milli> ms_timediff =
                std::chrono::high_resolution_clock::now() - start_;
            return ms_timediff.count();
        }
        else
        {
            throw std::runtime_error("Start the timer first.");
        }
    }
    void reset() { busy_ = false; }

   private:
    bool busy_;
    std::chrono::high_resolution_clock::time_point start_;
};

}  // namespace aaucns_rio

#endif /* TIMER_H_ */
