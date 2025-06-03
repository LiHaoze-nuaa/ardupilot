#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CANManager/AP_FOCCAN.h>
#include <AP_CANManager/AP_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;
uint8_t foccan_count;
 
void AP_FOCCAN::init(uint8_t driver_index, bool enable_filters) // 初始化
{   _driver_index = driver_index;
    if (_initialized) { return; } // 如果为真，则已经初始化过，直接返回，避免重复初始化
    if (!hal.scheduler->thread_create( // 创建一个新的线程，用于执行类成员函数update
    FUNCTOR_BIND_MEMBER(&AP_FOCCAN::update, void), _thread_name, 4096,
    AP_HAL::Scheduler::PRIORITY_MAIN, 1)) { return; } // 如果为假，则线程创建失败
    _initialized = true; // 已经成功初始化
    snprintf(_thread_name, sizeof(_thread_name), "FOCCAN_%u", driver_index);
    send_current(0); // 发送0电流指令
}
 
bool AP_FOCCAN::add_interface(AP_HAL::CANIface *can_iface) // 添加CAN接口
{   if (_can_iface != nullptr) { return false; } // 已经存在一个CAN接口，添加接口失败
    _can_iface = can_iface; // 将can_iface指针赋值给_can_iface，表示成功添加CAN接口
    if (_can_iface == nullptr) { return false; }
    if (!_can_iface->is_initialized()) { return false; }
    return true;
}

AP_FOCCAN *AP_FOCCAN::get_foccan(uint8_t driver_index) // 通过驱动器索引获取对象
{   if (driver_index >= AP::can().get_num_drivers() ||
    AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::FOCCAN) 
    { return nullptr; }
    return static_cast<AP_FOCCAN*>(AP::can().get_driver(driver_index));
}

void AP_FOCCAN::update()
{   AP_HAL::CANFrame txFrame{}; // 创建用于发送CAN帧的对象
    AP_HAL::CANFrame rxFrame{}; // 创建用于接收CAN帧的对象
    while (true) 
    {   if (!_initialized) { hal.scheduler->delay_microseconds(10000);
            continue; } // 如果未初始化，则延迟10毫秒，继续下一次循环
        #ifdef VECTOR_MODE // 矢量模式
        #else // 力矩摆模式
        send_current(target_current);
        #endif
        hal.scheduler->delay_microseconds(2500); // 延迟2.5毫秒，经测试，这里是很稳定的400Hz
        uint64_t timeout = AP_HAL::micros64() + 250UL; // 获取当前微秒数并加上250，得到超时时间
        while (read_frame(rxFrame, timeout)) // 查找CAN总线上的任何消息响应，读取CAN帧
        { if (rxFrame.id == 0x141) { handle_reply(rxFrame); } } } // 处理电机的测量数据
}

// read frame on CAN bus, returns true on succses
bool AP_FOCCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) 
{   if (!_initialized) { return false; }
    bool read_select = true;
    bool write_select = false;
    bool ret = _can_iface->select(read_select, write_select, nullptr, timeout); // 检查是否有数据可以读取
    if (!ret || !read_select) { return false; } // No frame available
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags{};
    return (_can_iface->receive(recv_frame, time, flags) == 1); // 接收数据并填充recv_frame
}

// write frame on CAN bus, returns true on success
bool AP_FOCCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) // 用于向CAN总线上发送帧
{   if (!_initialized) { return false; }
    bool read_select = false;
    bool write_select = true;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout); // 检查是否能够发送数据
    if (!ret || !write_select) { return false; }
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

void AP_FOCCAN::handle_reply(AP_HAL::CANFrame &frame) 
{   encoder_angle = static_cast<uint16_t>(frame.data[6] | frame.data[7] << 8 );
    pend_angle = 0.0056f * encoder_angle - 86.5899f; // 转换为角度，7440对应-45度, 23540对应45度
}

bool AP_FOCCAN::send_current(int16_t current) 
{   if(foccan_count >= 50) {
    gcs().send_text(MAV_SEVERITY_INFO, "current = %d, pend_angle = %.2f", current, pend_angle);
    foccan_count = 0; }
    foccan_count ++;
    uint8_t send_data[8] = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 预设默认值
    send_data[4] = static_cast<uint8_t>(current);       // 低字节
    send_data[5] = static_cast<uint8_t>(current >> 8);  // 高字节
    AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x141, send_data, 0x08); 
    uint64_t timeout_us = 2500UL;
    return write_frame(frame, timeout_us);
}

AP_FOCCAN *AP_FOCCAN::_singleton;

namespace AP { AP_FOCCAN *FOCCAN() { return AP_FOCCAN::get_singleton(); } } // AP命名空间