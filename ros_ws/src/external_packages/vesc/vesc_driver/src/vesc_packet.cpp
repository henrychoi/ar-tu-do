// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_packet.h"

#include <cassert>
#include <iterator>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "vesc_driver/vesc_packet_factory.h"
#include "vesc_driver/datatypes.h"

namespace vesc_driver
{

VescFrame::VescFrame(int payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256) {
    // single byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + payload_size));
    *frame_->begin() = 2;
    *(frame_->begin() + 1) = payload_size;
    payload_.first = frame_->begin() + 2;
  }
  else {
    // two byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + 1 + payload_size));
    *frame_->begin() = 3;
    *(frame_->begin() + 1) = payload_size >> 8;
    *(frame_->begin() + 2) = payload_size & 0xFF;
    payload_.first = frame_->begin() + 3;
  }

  payload_.second = payload_.first + payload_size;
  *(frame_->end() - 1) = 3;
}

VescFrame::VescFrame(const BufferRangeConst& frame, const BufferRangeConst& payload)
{
  /* VescPacketFactory::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
  assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
  assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
  assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
  assert(std::distance(frame.first, payload.first) > 0 &&
         std::distance(payload.second, frame.second) > 0);

  frame_.reset(new Buffer(boost::begin(frame), boost::end(frame)));
  payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
  payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
}

VescPacket::VescPacket(const std::string& name, int payload_size, int payload_id) :
  VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(boost::distance(payload_) > 0);
  *payload_.first = payload_id;
}

VescPacket::VescPacket(const std::string& name, boost::shared_ptr<VescFrame> raw) :
  VescFrame(*raw), name_(name)
{
}

/*------------------------------------------------------------------------------------------------*/

VescPacketFWVersion::VescPacketFWVersion(boost::shared_ptr<VescFrame> raw) :
  VescPacket("FWVersion", raw)
{
}

int VescPacketFWVersion::fwMajor() const
{
  return *(payload_.first + 1);
}

int VescPacketFWVersion::fwMinor() const
{
  return *(payload_.first + 2);
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)

VescPacketRequestFWVersion::VescPacketRequestFWVersion() :
  VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketValues::VescPacketValues(boost::shared_ptr<VescFrame> raw) :
  VescPacket("Values", raw)
{
}
// @return VESC mc_interface_temp_fet_filtered(): float16
double VescPacketValues::temp_pcb() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 5)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 6)));
//printf("temp_pcb int16 = %d\n", v);
  return static_cast<double>(v) / 1e1;
}
// @return VESC mc_interface_read_reset_avg_motor_current
double VescPacketValues::current_motor() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 7)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 8)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 9)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 10)));
  return static_cast<double>(v) / 1e2;
}
// @return VESC mc_interface_read_reset_avg_input_current
double VescPacketValues::current_in() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 11)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 12)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 13)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 14)));
  return static_cast<double>(v) / 100.0;
}
// @return VESC mc_interface_get_duty_cycle_now
double VescPacketValues::duty_now() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 15)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 16)));
  return static_cast<double>(v) / 1000.0;
}
// @return VESC mc_interface_get_rpm
double VescPacketValues::rpm() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 17)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 18)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 19)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 20)));
  return static_cast<double>(v);
}
// @return VESC GET_INPUT_VOLTAGE
double VescPacketValues::v_in() const
{
  int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 21)) << 8) +
                                   static_cast<uint16_t>(*(payload_.first + 22)));
  return static_cast<double>(v) / 10.0;
}
// @return VESC mc_interface_get_amp_hours
double VescPacketValues::amp_hours() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 23)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 24)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 25)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 26)));
  return static_cast<double>(v) / 10000.;
}
// @return VESC mc_interface_get_amp_hours_charged
double VescPacketValues::amp_hours_charged() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 27)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 28)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 29)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 30)));
  return static_cast<double>(v) / 10000.;
}
// @return VESC mc_interface_get_watt_hours
double VescPacketValues::watt_hours() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 31)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 32)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 33)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 34)));
  return static_cast<double>(v) / 10000;
}
// @return VESC mc_interface_get_watt_hours_charged
double VescPacketValues::watt_hours_charged() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 35)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 36)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 37)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 38)));
  return static_cast<double>(v) / 10000;
}
// @return VESC mc_interface_get_tachometer_value
double VescPacketValues::tachometer() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 39)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 40)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 41)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 42)));
  return static_cast<double>(v);
}
// @return VESC mc_interface_get_tachometer_abs_value
double VescPacketValues::tachometer_abs() const
{
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 43)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 44)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 45)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 46)));
  return static_cast<double>(v);
}
// @return mc_interface_get_fault
int VescPacketValues::fault_code() const
{
  return static_cast<int32_t>(*(payload_.first + 47));
}

REGISTER_PACKET_TYPE(COMM_GET_VALUES_SELECTIVE, VescPacketValues)

VescPacketRequestValues::VescPacketRequestValues() :
  VescPacket("VescPacketRequestValues", 5, COMM_GET_VALUES_SELECTIVE)
{
  // Hard-coding the quantities I will request, tailored for just what I need
  static constexpr uint32_t kMask =
    1 <<  0 | // mc_interface_temp_fet_filtered = temperature_pcb
    1 <<  2 | // mc_interface_read_reset_avg_motor_current = current_motor
    1 <<  3 | // mc_interface_read_reset_avg_input_current = current_input
    1 <<  6 | // mc_interface_get_duty_cycle_now = duty_cycle
    1 <<  7 | // mc_interface_get_rpm = speed
    1 <<  8 | // GET_INPUT_VOLTAGE = voltage_input
    1 <<  9 | // mc_interface_get_amp_hours = charge_drawn
    1 << 10 | // mc_interface_get_amp_hours_charged = charge_regen
    1 << 11 | // mc_interface_get_watt_hours = energy_drawn
    1 << 12 | // mc_interface_get_watt_hours_charged = energy_regen
    1 << 13 | // mc_interface_get_tachometer_value = displacement
    1 << 14 | // mc_interface_get_tachometer_abs_value = distance_traveled
    1 << 15 ; // mc_interface_get_fault = fault_code

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(kMask) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(kMask) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(kMask) >>  8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>( static_cast<uint32_t>(kMask)        & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/


VescPacketSetDuty::VescPacketSetDuty(double duty) :
  VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  /** @todo range check duty */

  int32_t v = static_cast<int32_t>(duty * 100000.0);
  printf("SetDuty %d\n", v);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrent::VescPacketSetCurrent(double current) :
  VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  int32_t v = static_cast<int32_t>(current * 1000.0);
  printf("SetCurrent %d\n", v);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake) :
  VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  int32_t v = static_cast<int32_t>(current_brake * 1000.0);
//printf("SetCurrentBrake %d\n", v);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetRPM::VescPacketSetRPM(double rpm) :
  VescPacket("SetRPM", 5, COMM_SET_RPM)
{
  int32_t v = static_cast<int32_t>(rpm);
  //printf("SetRPM %d\n", v);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetPos::VescPacketSetPos(double pos) :
  VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  int32_t v = static_cast<int32_t>(pos * 1000000.0);
//printf("SetPos %d\n", v);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetServoPos::VescPacketSetServoPos(float servo0, float servo1) :
  VescPacket("SetServoPos", 5, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  int16_t v0 = static_cast<int16_t>(servo0 * 1000.f)
      , v1 = static_cast<int16_t>(servo1 * 1000.f);
//printf("SetServoPos %d %d\n", v0, v1);
  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v0) >> 8) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>( static_cast<uint16_t>(v0) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint16_t>(v1) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>( static_cast<uint16_t>(v1) & 0xFF);

  VescFrame::CRC crc_calc;
  crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
  uint16_t crc = crc_calc.checksum();
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

} // namespace vesc_driver
