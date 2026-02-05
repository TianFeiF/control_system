#pragma once

#include <atomic>

/* 
 * 键盘控制交互变量 (原子操作确保线程安全)
 * g_selected_axis: 当前选中的轴 ID (1-9)
 * g_axis_dir: 运动方向 (-1/0/1)
 */
extern std::atomic<int> g_selected_axis;
extern std::atomic<int> g_axis_dir;

/*
 * 键盘监听线程函数
 * 参数 arg: 指向主程序运行标志位 (volatile int *run) 的指针
 *           用于在按下 'q' 键时停止主程序
 */
void *keyboard_thread_main(void *arg);
