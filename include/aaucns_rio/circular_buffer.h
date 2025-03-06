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

#ifndef _CIRCULAR_BUFFER_H_
#define _CIRCULAR_BUFFER_H_

#include <array>
#include <exception>

namespace aaucns_rio
{
// TODO(jan): Rename to FIFO.
template <typename TElement, int Size>
class CircularBuffer
{
    static_assert(Size > 0);

   public:
    CircularBuffer() : n_elements_(0) {}

    const TElement& at(int index) const { return this->operator[](index); }

    TElement& at(int index) { return this->operator[](index); }

    const TElement& operator[](int index) const
    {
        if (index < n_elements_)
        {
            return buffer_[index];
        }
        else
        {
            throw std::range_error("Out of range.");
        }
    }

    TElement& operator[](int index)
    {
        if (index < n_elements_)
        {
            return buffer_[index];
        }
        else
        {
            throw std::range_error("Out of range.");
        }
    }

    // Element is pushed in the first place and
    // all the rest is moved forwards. This way the
    // last element is popped out.
    void push_front(const TElement& element)
    {
        if (n_elements_ < Size)
        {
            ++n_elements_;
        }
        // Pop the Size-th element out and shift the rest towards the
        // end.
        for (int i = n_elements_ - 1; i > 0; --i)
        {
            buffer_[i] = buffer_[i - 1];
        }
        // Overwrite the ist element with the new one.
        buffer_[0] = element;
    }

    // Element is pushed in the last place and
    // all the rest is moved backwards. This way the
    // first element is popped out.
    void push_back(const TElement& element)
    {
        if (n_elements_ < Size)
        {
            buffer_[n_elements_] = element;
            ++n_elements_;
        }
        else
        {
            // Pop the Size-th element out and shift the rest towards the
            // end.
            for (int i = 0; i < n_elements_ - 1; ++i)
            {
                buffer_[i] = buffer_[i + 1];
            }
            // Overwrite the ist element with the new one.
            buffer_[n_elements_ - 1] = element;
        }
    }

    CircularBuffer& operator=(const CircularBuffer& circular_buffer)
    {
        if (circular_buffer.capacity() != capacity())
        {
            throw std::range_error("Buffers have different capacities.");
        }

        if (this == &circular_buffer)
        {
            return *this;
        }

        for (int i = 0; i < circular_buffer.size(); ++i)
        {
            buffer_[i] = circular_buffer[i];
        }
        n_elements_ = circular_buffer.size();

        return *this;
    }

    int size() const { return n_elements_; }

    bool empty() const { return (n_elements_ == 0); }
    bool full() const { return (n_elements_ == Size); }

    int capacity() const { return Size; }

    void reset() { n_elements_ = 0; }

   private:
    std::array<TElement, Size> buffer_;
    int n_elements_;
};

template <typename TElement, int Size>
inline std::ostream& operator<<(
    std::ostream& os, const CircularBuffer<TElement, Size>& circular_buffer)
{
    os << "circular buffer start" << std::endl;
    for (int i = 0; i < circular_buffer.size(); ++i)
    {
        os << "buffer at " << i << " : " << circular_buffer.at(i) << std::endl;
    }
    os << "circular buffer end" << std::endl;
    return os;
}

}  // namespace aaucns_rio

#endif /* CIRCULAR_BUFFER_H_ */
