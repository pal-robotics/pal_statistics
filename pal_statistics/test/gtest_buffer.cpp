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


#include <limits>
#include <cfloat>
#include <memory>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "gtest/gtest.h"
#include "pal_statistics/static_circular_buffer.hpp"

template<class T>
class MyAlloc
{
public:
  // type definitions
  typedef T value_type;
  typedef T * pointer;
  typedef const T * const_pointer;
  typedef T & reference;
  typedef const T & const_reference;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  // rebind allocator to type U
  template<class U>
  struct rebind
  {
    typedef MyAlloc<U> other;
  };

  // return address of values
  pointer address(reference value) const
  {
    return &value;
  }
  const_pointer address(const_reference value) const
  {
    return &value;
  }

  /* constructors and destructor
   * - nothing to do because the allocator has no state
   */
  MyAlloc() throw()
  {
  }
  MyAlloc(const MyAlloc &) throw()
  {
  }
  template<class U>
  MyAlloc(const MyAlloc<U> &) throw()
  {
  }
  ~MyAlloc() throw()
  {
  }

  // return maximum number of elements that can be allocated
  size_type max_size() const throw()
  {
    return std::numeric_limits<std::size_t>::max() / sizeof(T);
  }

  // allocate but don't initialize num elements of type T
  pointer allocate(size_type num, const void * = 0)
  {
    // print message and allocate memory with global new
    std::cerr << typeid(T).name() << " allocate " << num << " element(s)" <<
      " of size " << sizeof(T) << std::endl;
    pointer ret = (pointer)(::operator new(num * sizeof(T)));
    std::cerr << " allocated at: " << reinterpret_cast<void *>(ret) << std::endl;
    return ret;
  }

  // initialize elements of allocated storage p with value value
  void construct(pointer p, const T & value)
  {
    // initialize memory with placement new
    new (reinterpret_cast<void *>(p)) T(value);
  }

  // destroy elements of initialized storage p
  void destroy(pointer p)
  {
    // destroy objects by calling their destructor
    p->~T();
  }

  // deallocate storage p of deleted elements
  void deallocate(pointer p, size_type num)
  {
    // print message and deallocate memory with global delete
    std::cerr << typeid(T).name() << " deallocate " << num << " element(s)" <<
      " of size " << sizeof(T) << " at: " << reinterpret_cast<void *>(p) << std::endl;
    ::operator delete(reinterpret_cast<void *>(p));
  }
};

// return that all specializations of this allocator are interchangeable
template<class T1, class T2>
bool operator==(const MyAlloc<T1> &, const MyAlloc<T2> &) throw()
{
  return true;
}
template<class T1, class T2>
bool operator!=(const MyAlloc<T1> &, const MyAlloc<T2> &) throw()
{
  return false;
}

TEST(BufferTest, basicTest)
{
  pal_statistics::StaticCircularBuffer<int> buffer;
  buffer.set_capacity(5, 0);

  for (size_t i = 0; i < buffer.capacity(); ++i) {
    buffer.push_back() = i;
  }

  for (size_t i = 0; i < buffer.capacity(); ++i) {
    EXPECT_EQ(static_cast<int>(i), buffer.front());
    buffer.pop_front();
  }
}

TEST(BufferTest, buffer)
{
  typedef std::vector<int, MyAlloc<int>> MyVector;

  std::cout << "Resizing vector" << std::endl;
  MyAlloc<int> my_alloc;
  MyVector v(my_alloc);
  v.resize(1000);

  MyAlloc<MyVector> my_buffer_alloc;
  std::cout << "Creating circular buffer" << std::endl;
  pal_statistics::StaticCircularBuffer<MyVector, MyAlloc<MyVector>> buffer(10, v, my_buffer_alloc);

  ASSERT_EQ(buffer.size(), 0u);
  ASSERT_EQ(buffer.capacity(), 10u);

  std::cout << "Size of type " << sizeof(std::vector<int>) << " size of container " <<
    v.size() * sizeof(int) << std::endl;
  std::cout << "Pushing first elements of vector" << std::endl;
  for (size_t i = 0; i < 10; ++i) {
    buffer.push_back();
    ASSERT_EQ(i + 1, buffer.size());
  }

  std::cout << "Pushing second elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back();
    ASSERT_EQ(10u, buffer.size());
  }
  /*
    std::cout << "Increasing size of container" << std::endl;
    v.resize(1001);
    std::cout << "Pushing first elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    std::cout << "Pushing second elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }


    std::cout << "Reducing size of container" << std::endl;
    v.resize(100);
    std::cout << "Pushing first elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    std::cout << "Pushing second elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }


    std::cout << "Increasing size of container" << std::endl;
    v.resize(3000);
    std::cout << "Resizing and clearing buffer" << std::endl;
    buffer.clear();
    buffer.resize(10, v);
    buffer.clear();
    std::cout << "Pushing first elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    std::cout << "Pushing second elements of vector" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    std::cout << "Pushing and popping" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
      buffer.pop_front();
      buffer.push_back(v);
    }
  */
  std::cout << "Destructor" << std::endl;
}

TEST(BufferTest, circularBuffer)
{
  typedef std::vector<int, MyAlloc<int>> MyVector;


  std::cout << "Resizing vector" << std::endl;
  MyAlloc<int> my_alloc;
  MyVector v(my_alloc);
  v.resize(1000);

  MyAlloc<MyVector> my_buffer_alloc;
  std::cout << "Creating circular buffer" << std::endl;
  boost::circular_buffer<MyVector, MyAlloc<MyVector>> buffer(10, v, my_buffer_alloc);

  std::cout << "Size of type " << sizeof(std::vector<int>) << " size of container " <<
    v.size() * sizeof(int) << std::endl;
  std::cout << "Pushing first elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
    //    ASSERT_EQ(i+1, buffer.size());
  }

  std::cout << "Pushing second elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
    //    ASSERT_EQ(10, buffer.size());
  }

  std::cout << "Increasing size of container" << std::endl;
  v.resize(1001);
  std::cout << "Pushing first elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }

  std::cout << "Pushing second elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }


  std::cout << "Reducing size of container" << std::endl;
  v.resize(100);
  std::cout << "Pushing first elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }

  std::cout << "Pushing second elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }


  std::cout << "Increasing size of container" << std::endl;
  v.resize(3000);
  std::cout << "Resizing and clearing buffer" << std::endl;
  buffer.clear();
  buffer.resize(10, v);
  buffer.clear();
  std::cout << "Pushing first elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }

  std::cout << "Pushing second elements of vector" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.push_back(v);
  }

  std::cout << "Popping" << std::endl;
  for (int i = 0; i < 10; ++i) {
    buffer.pop_front();
  }

  std::cout << "Pushing after pop " << std::endl;
  for (int i = 0; i < 20; ++i) {
    buffer.push_back(v);
  }


  std::cout << "Destructor" << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
