// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef PAL_STATISTICS__STATIC_CIRCULAR_BUFFER_HPP_
#define PAL_STATISTICS__STATIC_CIRCULAR_BUFFER_HPP_

#include <memory>
#include <vector>

namespace pal_statistics
{
/**
 * @brief CircularBuffer implementation that does not perform allocations/deallocations
 * outside of the constructor, destructor and resize methods.
 *
 * The Buffer can only be used by reading/writing to the existing elements
 *
 * Cannot use boost::circular_buffer because popping a vector destroys it and
 * deallocates memory
 */
template<typename T, typename Allocator = std::allocator<T>>
class StaticCircularBuffer
{
public:
  typedef std::vector<T, Allocator> VectorType;

  StaticCircularBuffer(size_t max_size, const T & val, const Allocator & alloc = Allocator())
  : buffer_(alloc)
  {
    set_capacity(max_size, val);
  }

  StaticCircularBuffer()
  {
    begin_iterator_ = buffer_.begin();
    end_iterator_ = begin_iterator_;
  }

  /**
   * @brief clear Change the size of the buffer to 0 (not capacity)
   *              Only modifies internal iterators
   */
  void clear()
  {
    begin_iterator_ = buffer_.begin();
    end_iterator_ = begin_iterator_;
    full_ = false;
  }

  /**
   * @brief set_capacity Allocates memory for max_size copies of val
   * @param Resets the beginning and end iterators, which reduces the size of the buffer
   * to zero
   */
  void set_capacity(size_t max_size, const T & val)
  {
    buffer_.assign(max_size, val);
    clear();
  }


  size_t capacity() const
  {
    return buffer_.size();
  }

  size_t size() const
  {
    if (full_) {
      return capacity();
    } else if (begin_iterator_ <= end_iterator_) {
      return std::distance(begin_iterator_, end_iterator_);
    } else {
      return buffer_.size() - std::distance(end_iterator_, begin_iterator_);
    }
  }

  T & front()
  {
    if (!full_ && (begin_iterator_ == end_iterator_)) {
      throw std::runtime_error("Buffer is empty");
    }
    return *begin_iterator_;
  }

  /**
   * @brief push_back Increases the buffer size (not capacity) by one, and returns a
   * reference to the last item in the buffer. This item may have been used in the past
   *
   * If the buffer becomes full, the returned reference already contains an item
   */
  T & push_back()
  {
    auto old_it = end_iterator_;
    if (full_) {
      advance(begin_iterator_);
    }
    advance(end_iterator_);

    // Buffer at max capacity
    if (end_iterator_ == begin_iterator_) {
      full_ = true;
    }
    return *old_it;
  }

  /**
   * @brief pop_front Reduces buffer size by one, advancing the begin iterator
   */
  void pop_front()
  {
    if (!full_ && (begin_iterator_ == end_iterator_)) {
      throw std::runtime_error("Buffer is empty");
    }
    advance(begin_iterator_);
    full_ = false;
  }

  /**
   * @brief getBuffer Provides a reference to the internal data structure, use at your own
   * risk.
   * @return
   */
  VectorType & getBuffer()
  {
    return buffer_;
  }

private:
  void advance(typename VectorType::iterator & it, size_t distance = 1)
  {
    for (size_t i = 0; i < distance; ++i) {
      it++;
      if (it == buffer_.end()) {
        it = buffer_.begin();
      }
    }
  }
  VectorType buffer_;
  typename VectorType::iterator begin_iterator_;
  typename VectorType::iterator end_iterator_;
  bool full_;  // Disambiguates when end == begin because it's empty, or it's full
};
}  // namespace pal_statistics

#endif  // PAL_STATISTICS__STATIC_CIRCULAR_BUFFER_HPP_
