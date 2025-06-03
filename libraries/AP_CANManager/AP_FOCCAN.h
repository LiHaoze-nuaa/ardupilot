#pragma once
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_FOCCAN : public AP_CANDriver {
public:
    AP_FOCCAN() {
        if (_singleton != nullptr) { return; }
        _singleton = this;
    }

    CLASS_NO_COPY(AP_FOCCAN);
    static const struct AP_Param::GroupInfo var_info[];

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface *can_iface) override;
    float getPendAngle() { return pend_angle; } // 获取转速
    void setCurrent(int16_t _current) { target_current = _current; }
    int16_t getCurrent() { return target_current; } // 用于在调试时获取电流值
    bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);
    bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);
    void update();
    static AP_FOCCAN *get_foccan(uint8_t driver_index);
    static AP_FOCCAN *get_singleton() { return _singleton; }
 
private:
    static AP_FOCCAN *_singleton;

    AP_Int8 _num_poles;
    AP_HAL::CANIface *_can_iface;
    uint8_t _driver_index;
    bool _initialized;
    char _thread_name[16];
    void handle_reply(AP_HAL::CANFrame &frame);
    bool send_current(int16_t current);
    int16_t target_current;
    uint16_t encoder_angle;
    float pend_angle;
};

namespace AP { AP_FOCCAN *FOCCAN(); }