/*
  @file

  @author victor

  @copyright (c) 2018 PAL Robotics SL. All Rights Reserved
*/

#include <pal_statistics/static_circular_buffer.h>
#include <boost/circular_buffer.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <climits>
#include <cfloat>
#include <memory>
using namespace std;

namespace pal_statistics
{
template <class T>
class MyAlloc
{
public:
  // type definitions
  typedef T value_type;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef T& reference;
  typedef const T& const_reference;
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  // rebind allocator to type U
  template <class U>
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
  MyAlloc(const MyAlloc&) throw()
  {
  }
  template <class U>
  MyAlloc(const MyAlloc<U>&) throw()
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
  pointer allocate(size_type num, const void* = 0)
  {
    // print message and allocate memory with global new
    std::cerr << typeid(T).name() << " allocate " << num << " element(s)"
              << " of size " << sizeof(T) << std::endl;
    pointer ret = (pointer)(::operator new(num * sizeof(T)));
    std::cerr << " allocated at: " << (void*)ret << std::endl;
    return ret;
  }

  // initialize elements of allocated storage p with value value
  void construct(pointer p, const T& value)
  {
    // initialize memory with placement new
    new ((void*)p) T(value);
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
    std::cerr << typeid(T).name() << " deallocate " << num << " element(s)"
              << " of size " << sizeof(T) << " at: " << (void*)p << std::endl;
    ::operator delete((void*)p);
  }
};

// return that all specializations of this allocator are interchangeable
template <class T1, class T2>
bool operator==(const MyAlloc<T1>&, const MyAlloc<T2>&) throw()
{
  return true;
}
template <class T1, class T2>
bool operator!=(const MyAlloc<T1>&, const MyAlloc<T2>&) throw()
{
  return false;
}



TEST(BufferTest, basicTest)
{
  StaticCircularBuffer<int> buffer;
  buffer.set_capacity(5, 0);


  for (int i = 0; i < buffer.capacity(); ++i)
  {
    buffer.push_back() = i;
  }

  for (int i = 0; i < buffer.capacity(); ++i)
  {
    ROS_INFO_STREAM(i);
    EXPECT_EQ(i, buffer.front());
    buffer.pop_front();
  }
}

TEST(BufferTest, buffer)
{
  typedef std::vector<int, MyAlloc<int>> MyVector;



  ROS_INFO_STREAM("Resizing vector");
  MyAlloc<int> my_alloc;
  MyVector v(my_alloc);
  v.resize(1000);

  MyAlloc<MyVector> my_buffer_alloc;
  ROS_INFO_STREAM("Creating circular buffer");
  StaticCircularBuffer<MyVector, MyAlloc<MyVector>> buffer(10, v, my_buffer_alloc);

  ASSERT_EQ(buffer.size(), 0);
  ASSERT_EQ(buffer.capacity(), 10);

  ROS_INFO_STREAM("Size of type " << sizeof(std::vector<int>) << " size of container "
                                  << v.size() * sizeof(int));
  ROS_INFO_STREAM("Pushing first elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back();
    ASSERT_EQ(i + 1, buffer.size());
  }

  ROS_INFO_STREAM("Pushing second elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back();
    ASSERT_EQ(10, buffer.size());
  }
  /*
    ROS_INFO_STREAM("Increasing size of container");
    v.resize(1001);
    ROS_INFO_STREAM("Pushing first elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    ROS_INFO_STREAM("Pushing second elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }


    ROS_INFO_STREAM("Reducing size of container");
    v.resize(100);
    ROS_INFO_STREAM("Pushing first elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    ROS_INFO_STREAM("Pushing second elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }


    ROS_INFO_STREAM("Increasing size of container");
    v.resize(3000);
    ROS_INFO_STREAM("Resizing and clearing buffer");
    buffer.clear();
    buffer.resize(10, v);
    buffer.clear();
    ROS_INFO_STREAM("Pushing first elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    ROS_INFO_STREAM("Pushing second elements of vector");
    for (int i = 0; i < 10; ++i)
    {
      buffer.push_back(v);
    }

    ROS_INFO_STREAM("Pushing and popping");
    for (int i = 0; i < 10; ++i)
    {
      buffer.pop_front();
      buffer.push_back(v);
    }
  */
  ROS_WARN_STREAM("Destructor");
}

TEST(BufferTest, circularBuffer)
{
  typedef std::vector<int, MyAlloc<int>> MyVector;



  ROS_INFO_STREAM("Resizing vector");
  MyAlloc<int> my_alloc;
  MyVector v(my_alloc);
  v.resize(1000);

  MyAlloc<MyVector> my_buffer_alloc;
  ROS_INFO_STREAM("Creating circular buffer");
  boost::circular_buffer<MyVector, MyAlloc<MyVector>> buffer(10, v, my_buffer_alloc);

  ROS_INFO_STREAM("Size of type " << sizeof(std::vector<int>) << " size of container "
                                  << v.size() * sizeof(int));
  ROS_INFO_STREAM("Pushing first elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
    //    ASSERT_EQ(i+1, buffer.size());
  }

  ROS_INFO_STREAM("Pushing second elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
    //    ASSERT_EQ(10, buffer.size());
  }

  ROS_INFO_STREAM("Increasing size of container");
  v.resize(1001);
  ROS_INFO_STREAM("Pushing first elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }

  ROS_INFO_STREAM("Pushing second elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }


  ROS_INFO_STREAM("Reducing size of container");
  v.resize(100);
  ROS_INFO_STREAM("Pushing first elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }

  ROS_INFO_STREAM("Pushing second elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }


  ROS_INFO_STREAM("Increasing size of container");
  v.resize(3000);
  ROS_INFO_STREAM("Resizing and clearing buffer");
  buffer.clear();
  buffer.resize(10, v);
  buffer.clear();
  ROS_INFO_STREAM("Pushing first elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }

  ROS_INFO_STREAM("Pushing second elements of vector");
  for (int i = 0; i < 10; ++i)
  {
    buffer.push_back(v);
  }

  ROS_INFO_STREAM("Popping");
  for (int i = 0; i < 10; ++i)
  {
    buffer.pop_front();
  }

  ROS_INFO_STREAM("Pushing after pop ");
  for (int i = 0; i < 20; ++i)
  {
    buffer.push_back(v);
  }


  ROS_WARN_STREAM("Destructor");
}
}  // namespace pal_statistics

int main(int argc, char** argv)
{
  ros::init(argc, argv, "buffer_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
