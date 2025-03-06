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

#ifndef _ITERATION_H_
#define _ITERATION_H_

#include <aaucns_rio/debug.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace aaucns_rio
{
// *Not* to be used in multithread environment. Log from one callback at a time.
class Iteration
{
   public:
    Iteration(debug::Logger& logger) : logger_(logger)
    {
        logger_.startLoggingIteration();
    }

    ~Iteration() { logger_.stopLoggingIteration(); }

    // TODO(jan): implement similar function for std::vector.
    template <typename t>
    void writeEntry(const Eigen::MatrixBase<t>& matrix,
                    const std::string& entry_name)
    {
        logger_.writeEigenMatrixToFile(matrix, entry_name);
    }

   private:
    // Keep `Logger` object around as long as `Iteration` is alive.
    debug::Logger& logger_;
};
}  // namespace aaucns_rio

#endif /* _ITERATION_H_ */
