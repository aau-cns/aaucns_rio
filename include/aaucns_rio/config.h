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

#ifndef _CONFIG_H_
#define _CONFIG_H_

namespace aaucns_rio
{
class Config
{
   public:
    static constexpr bool kEnablePersistentFeatures = true;
    static constexpr bool kWriteFeaturesToFile = false;
    static constexpr bool kEnableLogging = false;
    static constexpr int kMaxPastElements = 10;
    static constexpr int kNSamplesForFeaturePersistence = 7;

    static_assert(kMaxPastElements > 0);
    static_assert(kEnablePersistentFeatures
                      ? (kNSamplesForFeaturePersistence <= kMaxPastElements)
                      : true);
};

class ConfigFg : public Config
{
   public:
    static constexpr int kNMaxIMUMeasurements = 512;
    static constexpr int kNPosesInWindow = 10;
};

}  // namespace aaucns_rio

#endif /* CONFIG_H_ */
