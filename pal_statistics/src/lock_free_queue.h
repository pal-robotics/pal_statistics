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

#ifndef LOCK_FREE_QUEUE_H
#define LOCK_FREE_QUEUE_H

#include <boost/lockfree/queue.hpp>
namespace pal_statistics
{
/**
 * @brief Simple wrapper around boost lockfree queue to keep track of the reserved memory
 *        Boost's implementation of reserve always increases the capacity by the specified
 * size
 */
template <typename T>
class LockFreeQueue : private boost::lockfree::queue<T>
{
public:
  typedef boost::lockfree::queue<T> BaseType;

  LockFreeQueue() : BaseType(0), reserved_size(0)
  {
  }
  void set_capacity(typename BaseType::size_type n)
  {
    long long missing_size = n - reserved_size;
    if (missing_size > 0)
    {
      BaseType::reserve(missing_size);
      reserved_size += missing_size;
    }
  }

  bool bounded_push(T const &t)
  {
    return BaseType::bounded_push(t);
  }

  template <typename U>
  bool pop(U &ret)
  {
    return BaseType::pop(ret);
  }

private:
  std::atomic<size_t> reserved_size;
};
} // pal_statistics
#endif // LOCK_FREE_QUEUE_H
