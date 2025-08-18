#ifndef UNIQUE_PTR_H
#define UNIQUE_PTR_H

#ifndef __AVR__

#include <memory>
#include <utility>

#else //std::unique_ptr and std::move shims for AVR

namespace std {

template<typename T> class unique_ptr {
private: T* ptr;
public:

  unique_ptr() : ptr(nullptr) {}
  explicit unique_ptr(T* p) : ptr(p) {}

  ~unique_ptr() { delete ptr; }

  //disable copy and assignment
  unique_ptr(const unique_ptr&) = delete;
  unique_ptr& operator=(const unique_ptr&) = delete;

  //enable ownership transfer
  unique_ptr(unique_ptr&& other) noexcept : ptr(other.ptr) { other.ptr = nullptr; }
  unique_ptr& operator=(unique_ptr&& other) noexcept {
    if (this != &other) {
      if (ptr) delete ptr;
      ptr = other.ptr;
      other.ptr = nullptr;
    }
    return *this;
  }

  void reset(T* p = nullptr) { delete ptr; ptr = p; }

  T* operator->() const { return ptr; }

  explicit operator bool() const { return ptr != nullptr; }
};

template<class T> struct remove_reference { typedef T Type; };
template<class T> struct remove_reference<T&> { typedef T Type; };
template<class T> struct remove_reference<T&&> { typedef T Type; };

template <typename T> typename remove_reference<T>::Type&& move(T&& arg) {
  return static_cast<typename remove_reference<T>::Type&&>(arg);
}

} //namespace std

#endif //__AVR__

#endif //UNIQUE_PTR_H
