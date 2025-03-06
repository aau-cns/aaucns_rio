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

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

namespace aaucns_rio
{
class Parameters
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Parameters() = default;

    double noise_acc_ = 0.083;
    double noise_accbias_ = 0.0083;
    double noise_gyr_ = 0.013;
    double noise_gyrbias_ = 0.0013;
    double noise_qri_ = 0.0;
    double noise_pri_ = 0.0;
    double noise_meas1_ = 0.05;
    double noise_meas2_ = 0.01;
    double noise_meas3_ = 0.01;
    double noise_meas4_ = 0.01;
    double noise_pf_x_ = 0.0;
    double noise_pf_y_ = 0.0;
    double noise_pf_z_ = 0.0;
    // Parameters not directly used in the implementation.
    double noise_qwv_ = 0.0;
    double noise_aux_ = 0.0;
    double noise_scale_ = 0.0;
};

}  // namespace aaucns_rio

#endif /* _PARAMETERS_H_ */
