/**
 *
 * MIT License
 * 
 * Copyright (c) 2019 PAL Robotics S.L.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
**/

#ifndef STATICCIRCULARBUFFER_H
#define STATICCIRCULARBUFFER_H

#include <memory>
#include <vector>
/**
 * @brief CircularBuffer implementation that does not perform allocations/deallocations
 * outside of the constructor, destructor and resize methods.
 *
 * The Buffer can only be used by reading/writing to the existing elements
 *
 * Cannot use boost::circular_buffer because popping a vector destroys it and
 * deallocates memory
 */
template <typename T, typename Allocator = std::allocator<T>>
class StaticCircularBuffer
{
public:
 typedef std::vector<T, Allocator> VectorType;

  StaticCircularBuffer(size_t max_size, const T& val, const Allocator& alloc = Allocator())
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
  void set_capacity(size_t max_size, const T& val)
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
    if (full_)
      return capacity();
    else if (begin_iterator_ <= end_iterator_)
      return std::distance(begin_iterator_, end_iterator_);
    else
      return buffer_.size() - std::distance(end_iterator_, begin_iterator_);
  }

  T& front()
  {
    if (!full_ && (begin_iterator_ == end_iterator_))
      throw std::runtime_error("Buffer is empty");
    return *begin_iterator_;
  }

  /**
   * @brief push_back Increases the buffer size (not capacity) by one, and returns a
   * reference to the last item in the buffer. This item may have been used in the past
   *
   * If the buffer becomes full, the returned reference already contains an item
   */
  T& push_back()
  {

    auto old_it = end_iterator_;
    if (full_)
      advance(begin_iterator_);
    advance(end_iterator_);

    // Buffer at max capacity
    if (end_iterator_ == begin_iterator_)
    {
      full_ = true;
    }
    return *old_it;
  }

  /**
   * @brief pop_front Reduces buffer size by one, advancing the begin iterator
   */
  void pop_front()
  {
   if (!full_ && (begin_iterator_ == end_iterator_))
     throw std::runtime_error("Buffer is empty");
    advance(begin_iterator_);
    full_ = false;
  }

  /**
   * @brief getBuffer Provides a reference to the internal data structure, use at your own
   * risk.
   * @return
   */
  VectorType &getBuffer()
  {
   return buffer_;
  }

private:
  void advance(typename VectorType::iterator& it, size_t distance = 1)
  {
    for (size_t i = 0; i < distance; ++i)
    {
      it++;
      if (it == buffer_.end())
        it = buffer_.begin();
    }
  }
  VectorType buffer_;
  typename VectorType::iterator begin_iterator_;
  typename VectorType::iterator end_iterator_;
  bool full_; //Disambiguates when end == begin because it's empty, or it's full
};

#endif // STATICCIRCULARBUFFER_H
