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

#ifndef _ID_MANAGER_H_
#define _ID_MANAGER_H_

#include <algorithm>
#include <array>
#include <vector>

namespace aaucns_rio
{
class IdManager
{
   private:
    struct Id
    {
        Id() { reset(); }
        void reset()
        {
            free = true;
            is_active = false;
            was_active = false;
            was_initialized = false;
        }
        bool free;
        bool is_active;
        bool was_active;
        bool was_initialized;
    };

   public:
    IdManager() { reset(); }

    void reset()
    {
        for (auto& element : ids_)
        {
            // Set true if element is free to take.
            element.reset();
        }
    }

    std::size_t getFreeId()
    {
        const auto is_free = [](const Id& id) {
            return ((id.free == true) && (id.was_active == false));
        };
        const auto iter =
            std::find_if(std::begin(ids_), std::end(ids_), is_free);
        std::size_t index = std::distance(std::begin(ids_), iter);
        // Not free anymore.
        ids_[index].free = false;
        ids_[index].is_active = true;
        ids_[index].was_initialized = false;
        return index;
    }

    void resetId(const std::size_t index) { ids_[index].reset(); }

    void releaseId(const std::size_t index_to_release)
    {
        ids_[index_to_release].free = true;
        ids_[index_to_release].is_active = false;
        ids_[index_to_release].was_active = true;
        ids_[index_to_release].was_initialized = false;
    }
    void releaseIds(const std::vector<std::size_t>& indexes_to_release)
    {
        for (int i = 0; i < indexes_to_release.size(); ++i)
        {
            releaseId(indexes_to_release[i]);
        }
    }

    void resetIdsExceptCurrentlyActive()
    {
        const std::vector<std::size_t> currently_active_ids =
            getCurrentActiveIds();
        reset();
        for (int i = 0; i < currently_active_ids.size(); ++i)
        {
            ids_[currently_active_ids[i]].free = false;
            ids_[currently_active_ids[i]].is_active = true;
            ids_[currently_active_ids[i]].was_initialized = true;
        }
    }

    std::vector<std::size_t> getCurrentActiveIds()
    {
        std::vector<std::size_t> indexes_vector;
        for (std::array<Id, kNMaxIds>::iterator iter = std::begin(ids_);
             iter != std::end(ids_); ++iter)
        {
            if (iter->is_active)
            {
                indexes_vector.push_back(std::distance(std::begin(ids_), iter));
            }
        }
        return indexes_vector;
    }

    std::vector<std::size_t> getAllInitializedIds()
    {
        std::vector<std::size_t> indexes_vector;
        for (std::array<Id, kNMaxIds>::iterator iter = std::begin(ids_);
             iter != std::end(ids_); ++iter)
        {
            if (iter->was_initialized)
            {
                indexes_vector.push_back(std::distance(std::begin(ids_), iter));
            }
        }
        return indexes_vector;
    }

    std::vector<std::size_t> getCurrentAndPastActiveIds()
    {
        std::vector<std::size_t> indexes_vector;
        for (std::array<Id, kNMaxIds>::iterator iter = std::begin(ids_);
             iter != std::end(ids_); ++iter)
        {
            if (iter->is_active || iter->was_active)
            {
                indexes_vector.push_back(std::distance(std::begin(ids_), iter));
            }
        }
        return indexes_vector;
    }

    bool hasIdBeenInitialized(const std::size_t index)
    {
        return ids_[index].was_initialized;
    }

    void initializeId(const std::size_t index)
    {
        ids_[index].was_initialized = true;
    }

   private:
    // Considered enough for holding all PFs.
    static constexpr std::size_t kNMaxIds = 512;
    std::array<Id, kNMaxIds> ids_;
};

}  // namespace aaucns_rio

#endif /* _ID_MANAGER_H_ */
