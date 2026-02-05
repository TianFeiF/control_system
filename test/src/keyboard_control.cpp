#include "keyboard_control.hpp"
#include <cstdio>
#include <termios.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

/* 定义全局交互变量 */
std::atomic<int> g_selected_axis{1};
std::atomic<int> g_axis_dir{0};

void *keyboard_thread_main(void *arg)
{
    volatile int *run_ptr = (volatile int *)arg;
    struct termios orig = {};
    if (tcgetattr(STDIN_FILENO, &orig) != 0) {
        while (*run_ptr) {
            usleep(1000);
        }
        return NULL;
    }

    struct termios raw = orig;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
    }

    int esc_state = 0;
    while (*run_ptr) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        const int rv = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (rv <= 0) {
            continue;
        }

        unsigned char buf[16];
        const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) {
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const unsigned char c = buf[i];
            if (esc_state == 1) {
                if (c == '[') {
                    esc_state = 2;
                } else {
                    esc_state = 0;
                }
                continue;
            }
            if (esc_state == 2) {
                if (c == 'D') {
                    g_axis_dir.store(-1);
                } else if (c == 'C') {
                    g_axis_dir.store(+1);
                } else if (c == 'A') {
                    g_axis_dir.store(+1);
                } else if (c == 'B') {
                    g_axis_dir.store(-1);
                }
                esc_state = 0;
                continue;
            }

            if (c >= '1' && c <= '9') {
                g_selected_axis.store((int)(c - '0'));
                continue;
            }
            if (c == '0') {
                g_selected_axis.store(0);
                g_axis_dir.store(0);
                continue;
            }
            if (c == ' ' || c == 's' || c == 'S') {
                g_axis_dir.store(0);
                continue;
            }
            if (c == 'q' || c == 'Q') {
                *run_ptr = 0;
                break;
            }

            if (c == 0x1b) {
                esc_state = 1;
                continue;
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    if (old_flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }
    return NULL;
}
