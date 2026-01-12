

#ifndef AD_BYD_PLANNING_UTILS_CIRCULAR_QUEUE_H
#define AD_BYD_PLANNING_UTILS_CIRCULAR_QUEUE_H

//#include <iostream>
// using namespace std;

#include <iterator>

#include "plan_common/log.h"
//#include <mutex>

namespace ad_byd {
namespace planning {

/*
 * 这是一个循环队列，但又与队列的特性有所不同
 * head_指示最新插入位置，tail_指示最后一个元素的后一个位置
 * begin()指向head_的内存地址，end()指向tail_的内存地址
 * 虽是循环队列，但要实现类似list的功能
 */
template <typename T>
class CircularQueue {
 private:
  T* buf_;
  int head_;
  int tail_;
  int capacity_;

 public:
  // explicit CircularQueue(int capacity) : buf_(new T[capacity]), head_(0),
  // tail_(0), capacity_(capacity) {};
  // 由于实现方式的缘故，设置的容量要比实际想申请的大1
  // 比如期望的循环队列大小为60，则下面的容量值应设置成61
  // 为保持与原数据结构的完全兼容，故不需要传参
  CircularQueue(int32_t capacity = 61)
      : buf_(new T[capacity]), head_(0), tail_(0), capacity_(capacity){};
  CircularQueue(const CircularQueue& other)
      : head_(other.head()), tail_(other.tail()), capacity_(other.capacity()) {
    buf_ = new T[capacity_];
    for (int i = 0; i < capacity_; ++i) {
      buf_[i] = other.buf()[i];
    }
  };

  virtual ~CircularQueue() { delete[] buf_; };

  bool full() const noexcept { return (head_ + 1) % capacity_ == tail_; }

  bool empty() const noexcept { return head_ == tail_; }

  int head() const noexcept { return head_; }

  int tail() const noexcept { return tail_; }

  T* buf() const noexcept { return buf_; }

  std::size_t capacity() const noexcept {
    return capacity_ - 1;  // 不存满，留一个来区别队列空和满的状态
  }

  std::size_t size() const noexcept {
    if (head_ >= tail_) {
      return head_ - tail_;
    } else {
      return capacity_ + head_ - tail_;
    }
  }

  void clear() noexcept { head_ = tail_ = 0; }

  void push_front(T&& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, push_front failed!");
    } else {
      head_ = (head_ + 1) % capacity_;
      buf_[head_] = value;
    }
  }

  void push_front(const T& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, push_front failed!");
    } else {
      head_ = (head_ + 1) % capacity_;
      buf_[head_] = value;
    }
  }

  void push_back(T&& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, push_back failed!");
    } else {
      buf_[tail_] = value;
      tail_ = (tail_ - 1 >= 0) ? (tail_ - 1) : (tail_ - 1 + capacity_);
    }
  }

  void push_back(const T& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, push_back failed!");
    } else {
      buf_[tail_] = value;
      tail_ = (tail_ - 1 >= 0) ? (tail_ - 1) : (tail_ - 1 + capacity_);
    }
  }

  void pop_front() {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is empty, pop_front failed!");
    }
    head_ = (head_ - 1 >= 0) ? (head_ - 1) : (head_ - 1 + capacity_);
  }

  void pop_back() {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is empty, pop_back failed!");
    }
    tail_ = (tail_ + 1) % capacity_;
  }

  T& front() {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Queue is empty, get front element failed!");
    }
    return buf_[head_];
  }

  const T& front() const {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Queue is empty, get front element failed!");
    }
    return buf_[head_];
  }

  T& back() {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Queue is empty, get back element failed!");
    }
    return buf_[(tail_ + 1) % capacity_];
  }

  const T& back() const {
    if (empty()) {
      // cout << "Queue is empty." << endl;
      LERROR("Circular queue is empty!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Queue is empty, get back element failed!");
    }
    return buf_[(tail_ + 1) % capacity_];
  }

  void emplace_front(const T& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, emplace_front failed!");
    } else {
      head_ = (head_ + 1) % capacity_;
      buf_[head_] = value;
    }
  }

  T& emplace_front(const T&& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, emplace_front failed!");
    } else {
      head_ = (head_ + 1) % capacity_;
      buf_[head_] = value;
    }
    return buf_[head_];
  }

  void emplace_back(const T& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, emplace_back failed!");
    } else {
      buf_[tail_] = value;
      tail_ = (tail_ - 1 >= 0) ? (tail_ - 1) : (tail_ - 1 + capacity_);
    }
  }

  T& emplace_back(const T&& value) {
    if (full()) {
      // cout << "Queue is full." << endl;
      LERROR("Circular queue is full!, size = %d, capicity = %d", size(),
             capacity());
      throw std::runtime_error("Circular queue is full, emplace_back failed!");
    } else {
      buf_[tail_] = value;
      tail_ = (tail_ - 1 >= 0) ? (tail_ - 1) : (tail_ - 1 + capacity_);
    }
    return buf_[(tail_ + 1) % capacity_];
  }

  class iterator : public std::iterator<std::forward_iterator_tag, T, long, T*,
                                        T&> {  // inheritance in order to fit
                                               // next() invocation
   private:
    CircularQueue<T>* q_;
    int index_;

   public:
    iterator(CircularQueue<T>* q, int index) : q_(q), index_(index) {}
    iterator(const iterator& other) : q_(other.q_), index_(other.index_) {}
    iterator& operator=(const iterator& other) {
      if (this != &other) {
        q_ = other.q_;
        index_ = other.index_;
      }
      return *this;
    }
    ~iterator() {}
    bool operator==(const iterator& other) const {
      return (q_ == other.q_ && index_ == other.index_);
    }
    bool operator!=(const iterator& other) const { return !(*this == other); }
    iterator& operator++() {
      index_ = (index_ - 1 >= 0) ? (index_ - 1) : (index_ - 1 + q_->capacity_);
      return *this;
    }
    iterator operator++(int) {
      iterator tmp(*this);
      ++(*this);
      return tmp;
    }
    T& operator*() const { return q_->buf_[index_]; }
    T* operator->() const { return &(q_->buf_[index_]); }
  };

  class reverse_iterator
      : public std::iterator<std::forward_iterator_tag, T, long, T*,
                             T&> {  // for rbegin() and rend()
   private:
    CircularQueue<T>* q_;
    int index_;

   public:
    reverse_iterator(CircularQueue<T>* q, int index) : q_(q), index_(index) {}
    reverse_iterator(const reverse_iterator& other)
        : q_(other.q_), index_(other.index_) {}
    reverse_iterator& operator=(const reverse_iterator& other) {
      if (this != &other) {
        q_ = other.q_;
        index_ = other.index_;
      }
      return *this;
    }
    ~reverse_iterator() {}
    bool operator==(const reverse_iterator& other) const {
      return (q_ == other.q_ && index_ == other.index_);
    }
    bool operator!=(const reverse_iterator& other) const {
      return !(*this == other);
    }
    reverse_iterator& operator++() {
      index_ = (index_ + 1) % q_->capacity_;
      return *this;
    }
    reverse_iterator operator++(int) {
      reverse_iterator tmp(*this);
      ++(*this);
      return tmp;
    }
    T& operator*() const { return q_->buf_[index_]; }
    T* operator->() const { return &(q_->buf_[index_]); }
  };

  class const_iterator
      : public std::iterator<std::forward_iterator_tag, T, long, T*, T&> {
   private:
    const CircularQueue<T>* cq_;
    int index_;

   public:
    const_iterator(const CircularQueue<T>* cq, int index)
        : cq_(cq), index_(index) {}
    const_iterator(const iterator& other)
        : cq_(other.cq_), index_(other.index_) {}
    const_iterator& operator=(const iterator& other) {
      if (this != &other) {
        cq_ = other.cq_;
        index_ = other.index_;
      }
      return *this;
    }
    ~const_iterator() {}
    bool operator==(const const_iterator& other) const {
      return (cq_ == other.cq_ && index_ == other.index_);
    }
    bool operator!=(const const_iterator& other) const {
      return !(*this == other);
    }
    const_iterator& operator++() {  // pre increment
      index_ = (index_ - 1 >= 0) ? (index_ - 1) : (index_ - 1 + cq_->capacity_);
      return *this;
    }
    const_iterator operator++(int) {  // post increment
      const_iterator tmp(*this);
      ++(*this);
      return tmp;
    }
    const T& operator*() const { return cq_->buf_[index_]; }

    const T* operator->() const { return &(cq_->buf_[index_]); }
  };

  class const_reverse_iterator
      : public std::iterator<  // for crbegin() and crend()
            std::forward_iterator_tag, T, long, T*, T&> {
   private:
    const CircularQueue<T>* cq_;
    int index_;

   public:
    const_reverse_iterator(const CircularQueue<T>* cq, int index)
        : cq_(cq), index_(index) {}
    const_reverse_iterator(const iterator& other)
        : cq_(other.cq_), index_(other.index_) {}
    const_reverse_iterator& operator=(const iterator& other) {
      if (this != &other) {
        cq_ = other.cq_;
        index_ = other.index_;
      }
      return *this;
    }
    ~const_reverse_iterator() {}
    bool operator==(const const_reverse_iterator& other) const {
      return (cq_ == other.cq_ && index_ == other.index_);
    }
    bool operator!=(const const_reverse_iterator& other) const {
      return !(*this == other);
    }
    const_reverse_iterator& operator++() {
      index_ = (index_ + 1) % cq_->capacity_;
      return *this;
    }
    const_reverse_iterator operator++(int) {
      const_reverse_iterator tmp(*this);
      ++(*this);
      return tmp;
    }
    const T& operator*() const { return cq_->buf_[index_]; }

    const T* operator->() const { return &(cq_->buf_[index_]); }
  };

  iterator begin() noexcept { return iterator(this, head_); }

  iterator end() noexcept { return iterator(this, tail_); }

  const_iterator begin() const noexcept { return const_iterator(this, head_); }

  const_iterator end() const noexcept { return const_iterator(this, tail_); }

  const_iterator cbegin() const noexcept { return const_iterator(this, head_); }

  const_iterator cend() const noexcept { return const_iterator(this, tail_); }

  reverse_iterator rbegin() noexcept {
    return reverse_iterator(this, (tail_ + 1) % (capacity() + 1));
  }

  reverse_iterator rend() noexcept {
    return reverse_iterator(this, (head_ + 1) % (capacity() + 1));
  }

  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(this, (tail_ + 1) % (capacity() + 1));
  }

  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(this, (head_ + 1) % (capacity() + 1));
  }

  const_reverse_iterator crbegin() const noexcept {
    return const_reverse_iterator(this, (tail_ + 1) % (capacity() + 1));
  }

  const_reverse_iterator crend() const noexcept {
    return const_reverse_iterator(this, (head_ + 1) % (capacity() + 1));
  }
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_CIRCULAR_QUEUE_H
