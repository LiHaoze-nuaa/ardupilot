#pragma once
#include <AP_CANManager/AP_CANDriver.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Param/AP_Param.h>

class AP_ASPCAN : public AP_CANDriver {
public:
  AP_ASPCAN() {
    if (_singleton != nullptr) { return; }
    _singleton = this;
  }

  CLASS_NO_COPY(AP_ASPCAN);
  static const struct AP_Param::GroupInfo var_info[];
  void init(uint8_t driver_index, bool enable_filters) override;
  bool add_interface(AP_HAL::CANIface *can_iface) override;
  float getairspeed(uint8_t id) { return airdata[id]; }
  float getairspeedorig(uint8_t id) { return airdata_orig[id]; }
  bool read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout);
  bool write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout);
  void update();
  static AP_ASPCAN *get_aspcan(uint8_t driver_index);
  static AP_ASPCAN *get_singleton() { return _singleton; }

  std::vector<float> last40ASP;
  std::vector<float> last80ASP;
  std::vector<float> last40SSA;
  std::vector<float> last80SSA;
  std::vector<float> last40AOA;
  std::vector<float> last80AOA;

private:
  static AP_ASPCAN *_singleton;

  AP_Int8 _num_poles;
  AP_HAL::CANIface *_can_iface;
  uint8_t _driver_index;
  bool _initialized;
  char _thread_name[16];
  void handle_data(AP_HAL::CANFrame &frame);
  bool send_command();
  float airdata_orig[7];
  float airdata_scal[7];
  float airdata_last[7];
  float airdata[7];
};

namespace AP { AP_ASPCAN *ASPCAN(); 
};