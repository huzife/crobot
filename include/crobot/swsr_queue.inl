template <typename T>
SWSR_Queue<T>::SWSR_Queue(std::size_t size)
    : size_(size + 1),
      head_(0),
      tail_(0) {
    buf_ = new T[size_];
}

template <typename T>
SWSR_Queue<T>::~SWSR_Queue() {
    if (buf_) {
        delete[] buf_;
        buf_ = nullptr;
    }
}

template <typename T>
bool SWSR_Queue<T>::push(T val) {
    auto next_tail = next_index(tail_);

    // full
    if (next_tail == head_)
        return false;

    // push element
    buf_[tail_] = val;
    tail_ = next_tail;

    return true;
}

template <typename T>
bool SWSR_Queue<T>::pop(T& val) {
    // empty
    if (head_ == tail_)
        return false;

    // pop element
    val = buf_[head_];
    head_ = next_index(head_);

    return true;
}

template <typename T>
size_t SWSR_Queue<T>::next_index(std::size_t index) const {
    return (index + 1) % size_;
}
