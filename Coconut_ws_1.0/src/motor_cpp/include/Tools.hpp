#ifndef __TOOLS_HPP__
#define __TOOLS_HPP__

#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>
#include <cstddef>
#include <thread>
#include <atomic>
#include <functional>


namespace keyboard{
    void setup_terminal(void);
    char read_key(void);
}

namespace Tools{

    /**
     * @brief 计时器
     */
    class MicroTimer {
    private:
        std::chrono::steady_clock::time_point start_time;
        
    public:
        MicroTimer();

        // 复位秒表
        void reset();

        // 获取秒表当前时间(us)
        long long elapsed_us() const ;
        
        // 获取秒表当前时间(ms)
        long long elapsed_ms() const ;
    };


    /**
     * @brief 线程共享变量
     */
    template <typename T>
    class SharedVar {
    private:
        std::atomic<T> value;
    public:
        SharedVar(T init = T()) : value(init) {}

        void set(T val) { value.store(val); }
        T get() const { return value.load(); }
        void increment() { value.fetch_add(1); }
    };

    /**
     * @brief 周期性任务模块
     */
    class PeriodicTask {
    public:
        PeriodicTask(std::function<void()> func, int freqUs);
        
        void start();
        void stop();
        
        // 析构函数
        ~PeriodicTask();
    private:
        std::function<void()> taskFunc;
        int intervalUs;
        std::atomic<bool> running;
        std::thread workerThread;

        // 线程入口函数
        void runLoop();
    };


    /**
     * @brief 环形缓冲区
     */
    template <typename T>
    class RingBuffer {
    public:
        explicit RingBuffer(size_t depth);

        void push(const T& data);
        bool pop(T& outData);

        size_t size() const;
        bool empty() const;
        bool full() const;
    private:
        std::vector<T> buffer;      // 底层存储容器
        size_t capacity;            // 缓冲区深度（容量）
        size_t readIndex;           // 读指针
        size_t writeIndex;          // 写指针
        size_t count;               // 当前数据量
        mutable std::mutex mtx;     // 互斥锁 (mutable 允许在 const 函数中加锁)

        // 最后一次成功读取的数据（用于空读保持）
        T lastValidData;            
        bool hasData;               // 是否曾经有过数据（防止初始值歧义）
    };

    template <typename T>
    RingBuffer<T>::RingBuffer(size_t depth) 
        : capacity(depth), readIndex(0), writeIndex(0), count(0), hasData(false) {
        buffer.resize(capacity);
    }

    template <typename T>
    void RingBuffer<T>::push(const T& data) {
        std::lock_guard<std::mutex> lock(mtx);

        // 写入数据
        buffer[writeIndex] = data;
        
        // 移动写指针
        writeIndex = (writeIndex + 1) % capacity;
        hasData = true; // 标记已经有数据了

        if (count == capacity) {
            readIndex = (readIndex + 1) % capacity;
        } else {
            count++;
        }
    }

    template <typename T>
    bool RingBuffer<T>::pop(T& outData) {
        std::lock_guard<std::mutex> lock(mtx);

        if (count > 0) {
            outData = buffer[readIndex];
            readIndex = (readIndex + 1) % capacity;
            count--;
            
            lastValidData = outData;
            return true;
        } else {
            if (hasData) {
                outData = lastValidData;
            } else {
                outData = T(); 
            }
            return false;
        }
    }

    template <typename T>
    size_t RingBuffer<T>::size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return count;
    }

    template <typename T>
    bool RingBuffer<T>::empty() const {
        std::lock_guard<std::mutex> lock(mtx);
        return count == 0;
    }

    template <typename T>
    bool RingBuffer<T>::full() const {
        std::lock_guard<std::mutex> lock(mtx);
        return count == capacity;
    }

} // namespace Tools







#endif