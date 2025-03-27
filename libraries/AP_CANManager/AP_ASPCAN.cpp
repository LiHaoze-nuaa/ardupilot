#include <AP_CANManager/AP_ASPCAN.h>
#include <AP_CANManager/AP_CAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <stdio.h>
#include <numeric>

extern const AP_HAL::HAL &hal;

void AP_ASPCAN::init(uint8_t driver_index, bool enable_filters) // 初始化
{   _driver_index = driver_index;
    if (_initialized) { return; } // 如果为真，则已经初始化过，直接返回，避免重复初始化
    if (!hal.scheduler->thread_create( // 创建一个新的线程，用于执行类成员函数update
    FUNCTOR_BIND_MEMBER(&AP_ASPCAN::update, void), _thread_name, 4096,
    AP_HAL::Scheduler::PRIORITY_MAIN, 1)) { return; } // 如果为假，则线程创建失败
    _initialized = true; // 已经成功初始化，这行之前不要插入其他程序
    snprintf(_thread_name, sizeof(_thread_name), "ASPCAN_%u", driver_index); 
    send_command(); // 用于发送循环采集指令 
}

bool AP_ASPCAN::add_interface(AP_HAL::CANIface *can_iface) // 添加CAN接口
{   if (_can_iface != nullptr) { return false; } // 已经存在一个CAN接口，添加接口失败
    _can_iface = can_iface; // 将can_iface指针赋值给_can_iface，表示成功添加CAN接口
    if (_can_iface == nullptr) { return false; }
    if (!_can_iface->is_initialized()) { return false; }
    return true;
}

AP_ASPCAN *AP_ASPCAN::get_aspcan(uint8_t driver_index) // 通过驱动器索引获取对象
{   if (driver_index >= AP::can().get_num_drivers() ||
    AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::ASPCAN) 
    { return nullptr; }
    return static_cast<AP_ASPCAN*>(AP::can().get_driver(driver_index));
}

void AP_ASPCAN::update() 
{   AP_HAL::CANFrame txFrame{}; // 创建用于发送的CAN帧
    AP_HAL::CANFrame rxFrame{}; // 创建用于接收的CAN帧
    while (true) 
    {   if (!_initialized) { hal.scheduler->delay_microseconds(10000);
            continue; } // 如果未初始化，则延迟10ms，继续下一次循环
        hal.scheduler->delay_microseconds(2500); // 延迟2.5ms，经验证，这里是很稳定的400Hz
        uint64_t timeout = AP_HAL::micros64() + 2500ULL; // 设置2.5ms超时
        while (read_frame(rxFrame, timeout)) // 查找CAN总线上的任何消息响应，读取CAN帧
        {   if (rxFrame.id == 0x100 ) { handle_data(rxFrame); } } // 处理接收的数据

        // 空速自适应滤波
        last40ASP.push_back(airdata_orig[3]);
        last80ASP.push_back(airdata_orig[3]);
        if (last40ASP.size() > 40) last40ASP.erase(last40ASP.begin()); // 保持最近40个数据
        if (last80ASP.size() > 80) last80ASP.erase(last80ASP.begin()); // 保持最近80个数据
        float sum40ASP = std::accumulate(last40ASP.begin(), last40ASP.end(), 0.0f);
        float sum80ASP = std::accumulate(last80ASP.begin(), last80ASP.end(), 0.0f);
        float avg40ASP = sum40ASP / last40ASP.size();
        float avg80ASP = (sum80ASP - sum40ASP) / (last80ASP.size() - last40ASP.size());
        float diffASP = avg80ASP - avg40ASP;
        if (diffASP < 0) { diffASP = -diffASP; } // 使用条件判断计算差值的绝对值
        // 根据diff调整factorASP，diff越大，factorASP越大，越尊重原数值，越激进，0.01<factorASP<0.5
        float factorASP = std::max(0.01f, std::min(0.5f, diffASP * 0.01f)); // 经调试，参数已处于最佳状态
        airdata[3] = factorASP * airdata_orig[3] + (1.0f - factorASP) * airdata_last[3]; // 对空速滤波
        
        // 侧滑角自适应滤波
        // 在空速较小(0~5m/s)时，抑制侧滑角的大幅波动；如果空速数据质量不佳，则侧滑角为0
        float scal_factor0;
        if (airdata_orig[0] < -28 || airdata_orig[0] > 28 || airdata_orig[1] < -28 || airdata_orig[1] > 28) { scal_factor0 = 0.0f; }
        else { scal_factor0 = (std::max(3.0f, std::min(5.0f, airdata[3])) - 3.0f )* 0.5f; }
        airdata_scal[0] = airdata_orig[0] * scal_factor0;
        last40SSA.push_back(airdata_scal[0]);
        last80SSA.push_back(airdata_scal[0]);
        if (last40SSA.size() > 40) last40SSA.erase(last40SSA.begin());
        if (last80SSA.size() > 80) last80SSA.erase(last80SSA.begin());
        float sum40SSA = std::accumulate(last40SSA.begin(), last40SSA.end(), 0.0f);
        float sum80SSA = std::accumulate(last80SSA.begin(), last80SSA.end(), 0.0f);
        float avg40SSA = sum40SSA / last40SSA.size();
        float avg80SSA = (sum80SSA - sum40SSA) / (last80SSA.size() - last40SSA.size());
        float diffSSA = avg80SSA - avg40SSA;
        if (diffSSA < 0) { diffSSA = -diffSSA; }
        float factorSSA = std::max(0.01f, std::min(0.5f, diffSSA * 0.018f)); // 原增益为0.01f, 后来设为0.02f噪声比较剧烈
        // 经调试，参数已处于最佳状态。如果factorSSA过小，会导致滤波后的数据严重延迟
        airdata[0] = factorSSA * airdata_scal[0] + (1.0f - factorSSA) * airdata_last[0]; // 滤波

        // 迎角自适应滤波   
        float scal_factor1 = (std::max(1.0f, std::min(3.0f, airdata[3])) - 1.0f )* 0.5f;
        airdata_scal[1] = airdata_orig[1] * scal_factor1; // 在空速较小(0~3m/s)时，抑制迎角的大幅波动
        last40AOA.push_back(airdata_scal[1]);
        last80AOA.push_back(airdata_scal[1]);
        if (last40AOA.size() > 40) last40AOA.erase(last40AOA.begin());
        if (last80AOA.size() > 80) last80AOA.erase(last80AOA.begin());
        float sum40AOA = std::accumulate(last40AOA.begin(), last40AOA.end(), 0.0f);
        float sum80AOA = std::accumulate(last80AOA.begin(), last80AOA.end(), 0.0f);
        float avg40AOA = sum40AOA / last40AOA.size();
        float avg80AOA = (sum80AOA - sum40AOA) / (last80AOA.size() - last40AOA.size());
        float diffAOA = avg80AOA - avg40AOA;
        if (diffAOA < 0) { diffAOA = -diffAOA; }
        float factorAOA = std::max(0.01f, std::min(0.5f, diffAOA * 0.012f)); // 原增益为0.01f, 后来设为0.015f效果比较激进
        airdata[1] = factorAOA * airdata_scal[1] + (1.0f - factorAOA) * airdata_last[1]; // 滤波

        // 探针偏转平方反馈控制
        float smoothAOA = 0.007 * airdata_scal[1] + 0.993 * airdata_last[1]; // 平滑滤波
        float normAOA = constrain_float(smoothAOA*0.04f, -1.0f, 1.0f); // 缩放
        float signAOA = normAOA > 0?1:-1;
        float powAOA = signAOA * powf(fabsf(normAOA), 2.0);
        if (airdata[3] < 2.0f) { tilt_probe_cmd = 0.0f; } 
        else { tilt_probe_cmd = tilt_probe_cmd - powAOA * 0.005f; } // 原本是0.0025f
        tilt_probe_cmd = constrain_float(tilt_probe_cmd, -0.6f, 0.6f); // 限幅，防止探针与气流方向正交
  
        airdata_last[0] = airdata[0];
        airdata_last[1] = airdata[1];
        airdata_last[3] = airdata[3];
    } 
}

// read frame on CAN bus, returns true on succses
bool AP_ASPCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout) 
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
bool AP_ASPCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout) // 用于向CAN总线上发送帧
{   if (!_initialized) { return false; }
    bool read_select = false;
    bool write_select = true;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, timeout); // 检查是否能够发送数据
    if (!ret || !write_select) { return false; }
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

void AP_ASPCAN::handle_data(AP_HAL::CANFrame &frame) // 处理接收的数据
{
    uint32_t data_id = frame.data[0];  // 数据标识符, 如 0x00, 0x01, 0x02 等
    if (data_id < 7) {      
        float value; // 使用memcpy将小端格式的float数据复制到airdata数组中
        memcpy(&value, &frame.data[1], sizeof(float));  // 小端数据存储
        airdata_orig[data_id] = value; }
        // airdata[3] += 0.1f; } // 用于检查调用频率
}

bool AP_ASPCAN::send_command() 
{   
    uint8_t send_buffer[8] = {0xF0, 0xFF, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00}; // 循环采集指令，实测最大帧率为25帧
 // uint8_t send_buffer[8] = {0xF0, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // 单次采集指令
    AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x100, send_buffer, 0x08);  // 长度为8字节
    uint64_t timeout_us = 10000UL; // 设置超时
    return write_frame(frame, timeout_us); // 发送CAN帧
}

AP_ASPCAN *AP_ASPCAN::_singleton;

namespace AP { AP_ASPCAN *ASPCAN() { return AP_ASPCAN::get_singleton(); 
} }; // namespace AP