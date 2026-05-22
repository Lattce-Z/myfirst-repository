#include "Tools.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

namespace Tools
{

MicroTimer::MicroTimer() { reset(); }

void MicroTimer::reset()
{
    start_time = std::chrono::steady_clock::now();
}

long long MicroTimer::elapsed_us() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
}

long long MicroTimer::elapsed_ms() const
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}



// 构造函数
PeriodicTask::PeriodicTask(std::function<void()> func, int freqUs)
    : taskFunc(func), intervalUs(freqUs), running(false) {
}

// 核心循环逻辑
void PeriodicTask::runLoop() {
    auto nextRunTime = std::chrono::steady_clock::now();

    while (running.load()) {
        try {
            taskFunc(); // 执行用户传入的函数
        } catch (const std::exception& e) {
            std::cerr << "[Error] Task Exception: " << e.what() << std::endl;
        }

        // 计算下一次运行时间
        nextRunTime += std::chrono::microseconds(intervalUs);
        
        // 休眠直到下一次运行
        std::this_thread::sleep_until(nextRunTime);
    }
}

// 启动线程
void PeriodicTask::start() {
    if (!running.load()) {
        running.store(true);
        // 创建线程并绑定成员函数
        workerThread = std::thread(&PeriodicTask::runLoop, this);
        std::cout << "[System] 线程已启动，频率: " << intervalUs << "us" << std::endl;
    }
}

// 停止线程
void PeriodicTask::stop() {
    if (running.load()) {
        running.store(false);
        if (workerThread.joinable()) {
            workerThread.join(); // 阻塞等待线程结束
        }
        std::cout << "[System] 线程已停止" << std::endl;
    }
}

// 析构函数
PeriodicTask::~PeriodicTask() {
    stop();
}

} // namespace Tools



namespace keyboard{

static struct termios saved_termios;
static int termios_saved = 0;


void restore_terminal() {
    if (termios_saved) {
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios);
    }
}

void signal_handler(int sig) {
    restore_terminal();
    exit(sig);
}

void setup_terminal() {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    saved_termios = tty;
    termios_saved = 1;

    tty.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);

    // 注册清理函数
    atexit(restore_terminal);
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
}

char read_key() {
    char c;
    read(STDIN_FILENO, &c, 1);
    return c;
}

} // namespace keyboard