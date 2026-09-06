#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>
struct Field {const char *name; uint16_t address; uint16_t size;};
int main(int argc, char **) {
  auto *port = dynamixel::PortHandler::getPortHandler("/dev/ttyACM0");
  auto *packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  if (!port->openPort() || !port->setBaudRate(1000000)) return 1;
  uint16_t model = 0; uint8_t error = 0;
  int result = packet->ping(port, 200, &model, &error);
  std::cout << "ping result=" << result << " model=" << model << " error=" << int(error) << std::endl;
  if (result || error || model != 20480) {port->closePort(); return 2;}
  if (argc == 2) {
    // Firmware's refresh-only trigger: reads servo goals into the proxy cache.
    result = packet->write1ByteTxRx(port,200,222,1,&error);
    std::cout << "refresh_goal_position addr=222 result=" << result << " error=" << int(error) << std::endl;
    if (result || error) {port->closePort(); return 3;}
  }
  Field fields[] = {{"millis",10,4},{"connect_ros",15,1},{"connect_manipulator",16,1},{"device_status",18,1},{"heartbeat",19,1},{"battery",42,4},{"connect_wheels",148,1},{"torque_wheels",149,1},{"torque_joints",199,1},{"goal_j1",200,4},{"goal_j2",204,4},{"goal_j3",208,4},{"goal_j4",212,4},{"goal_gripper",216,4},{"trigger_gripper",221,1},{"present_j1",224,4},{"present_j2",228,4},{"present_j3",232,4},{"present_j4",236,4},{"present_gripper",240,4},{"velocity_gripper",260,4},{"current_gripper",272,2},{"accel_gripper",300,4},{"profile_gripper",324,4},{"goal_current_gripper",340,2}};
  for (int sample=0; sample<3; ++sample) {
    std::cout << "sample=" << sample << std::endl;
    for (const auto &f:fields) {
      uint8_t data[4] = {}; error = 0;
      result = packet->readTxRx(port,200,f.address,f.size,data,&error);
      uint32_t value=0; for(int i=0;i<f.size;++i) value |= uint32_t(data[i]) << (8*i);
      std::cout << f.name << " address=" << f.address << " raw=" << value << " result=" << result << " error=" << int(error);
      if(f.address==42) {std::cout << " volts=" << value * 0.01;}
      if(f.address==272) std::cout << " signed=" << int16_t(value);
      std::cout << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  // Read-only ping of gripper servo ID on USB; no bus mode changes.
  result=packet->ping(port,15,&model,&error);
  std::cout << "direct_id15 result=" << result << " error=" << int(error) << std::endl;
  port->closePort(); delete port;
}
