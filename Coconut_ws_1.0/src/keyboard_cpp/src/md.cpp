#include <iostream>
#include "md.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

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
    if(read(STDIN_FILENO, &c, 1) > 0)
        return c;
    return 0;
}

}
