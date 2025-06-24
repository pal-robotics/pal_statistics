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


#ifndef LOCK_FREE_QUEUE_HPP_
#define LOCK_FREE_QUEUE_HPP_

#include "boost/lockfree/queue.hpp"

namespace pal_statistics
{
/**
 * @brief Simple wrapper around boost lockfree queue to keep track of the reserved memory
 *        Boost's implementation of reserve always increases the capacity by the specified
 * size
 */
template<typename T>
class LockFreeQueue : private boost::lockfree::queue<T>
{
public:
  typedef boost::lockfree::queue<T> BaseType;

  LockFreeQueue()
  : BaseType(0), reserved_size(0)
  {
  }
  void set_capacity(typename BaseType::size_type n)
  {
    int64_t missing_size = n - reserved_size;
    if (missing_size > 0) {
      BaseType::reserve(missing_size);
      reserved_size += missing_size;
    }
  }

  bool bounded_push(T const & t)
  {
    return BaseType::bounded_push(t);
  }

  template<typename U>
  bool pop(U & ret)
  {
    return BaseType::pop(ret);
  }

private:
  std::atomic<size_t> reserved_size;
};
}  // namespace pal_statistics
#endif  // LOCK_FREE_QUEUE_HPP_
