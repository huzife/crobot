#ifndef CROBOT_SWSR_QUEUE_H
#define CROBOT_SWSR_QUEUE_H

#include <cstddef>

/// @class SWSR_Queue
/// @brief A thread-safe, single-write and single-read queue,
///        implemented with ring buffer.
/// @tparam T element type
template <typename T>
class SWSR_Queue {
public:
    SWSR_Queue() = delete;
    SWSR_Queue(const SWSR_Queue&) = delete;
    SWSR_Queue& operator=(const SWSR_Queue&) = delete;
    SWSR_Queue(SWSR_Queue&&) = delete;
    SWSR_Queue& operator=(SWSR_Queue&&) = delete;

    SWSR_Queue(std::size_t size);
    ~SWSR_Queue();

    /// @brief Push an element if not full
    /// @param[in] val The element to be pushed
    /// @return bool
    /// @retval true    Success
    ///         false   Failed, queue is full
    bool push(T val);

    /// @brief Pop an element if not empty
    /// @param[out] val The element to be popped
    /// @return bool
    /// @retval true    Success
    ///         false   Failed, queue is empty
    bool pop(T& val);

private:
    std::size_t next_index(std::size_t index) const;

private:
    std::size_t head_;
    std::size_t tail_;
    std::size_t size_;
    T* buf_;
};

#include "swsr_queue.inl"

#endif // CROBOT_SWSR_QUEUE_H
