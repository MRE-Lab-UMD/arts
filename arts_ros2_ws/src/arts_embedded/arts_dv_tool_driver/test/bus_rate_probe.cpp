// Bus-rate probe, 2026-11-04 (docs/modules/dv_tool_driver.md, "Bus rate"). Standalone, not
// built by colcon: disc_drive's cycle (Fast Sync Read of present current/velocity/position +
// Sync Write of goal velocity 0) on IDs 1-5, flat out. Refuses unless every servo's torque is
// off. Build against the pinned SDK in oem_ros2_ws:
//   I=$ARTS_LOCAL/middle_overlay_ros2_wss/oem_ros2_ws/install/dynamixel_sdk
//   g++ -O2 -std=c++17 -I$I/include/dynamixel_sdk -I$I/include bus_rate_probe.cpp \
//       -L$I/lib -ldynamixel_sdk -Wl,-rpath,$I/lib -o bus_rate_probe
//   ./bus_rate_probe <port> <baud> [cycles]          measure
//   ./bus_rate_probe <port> <baud> --write <addr> <val>  one byte on all five, at the CURRENT
//       baud (EEPROM 8 Baud Rate: 3 = 1M, 4 = 2M; EEPROM 9 Return Delay Time, 2 us units)
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace dynamixel;
int main(int argc, char ** argv) {
  const char * port = argv[1]; int baud = atoi(argv[2]);
  PortHandler * ph = PortHandler::getPortHandler(port);
  PacketHandler * pk = PacketHandler::getPacketHandler(2.0);
  if (!ph->openPort() || !ph->setBaudRate(baud)) {printf("open failed\n"); return 1;}
  uint8_t err;
  for (int id = 1; id <= 5; ++id) {
    uint8_t t = 9;
    int r = COMM_TX_FAIL;
    for (int k = 0; k < 3 && r != COMM_SUCCESS; ++k) r = pk->read1ByteTxRx(ph, id, 64, &t, &err);
    if (r != COMM_SUCCESS || t != 0) {printf("id %d: torque read %d val %d, refusing\n", id, r, t); return 1;}
  }
  if (argc > 5 && !strcmp(argv[3], "--write")) {
    int addr = atoi(argv[4]), val = atoi(argv[5]);
    for (int id = 1; id <= 5; ++id)
      printf("id %d: addr %d -> %d: %s\n", id, addr, val,
             pk->getTxRxResult(pk->write1ByteTxRx(ph, id, addr, val, &err)));
    return 0;
  }
  int n = argc > 3 ? atoi(argv[3]) : 3000;
  GroupFastSyncRead rd(ph, pk, 126, 10);
  GroupSyncWrite wr(ph, pk, 104, 4);
  uint8_t zero[4] = {0, 0, 0, 0};
  for (int id = 1; id <= 5; ++id) {rd.addParam(id); wr.addParam(id, zero);}
  std::vector<double> dt; int fails = 0;
  auto prev = std::chrono::steady_clock::now();
  for (int i = 0; i < n; ++i) {
    if (rd.txRxPacket() != COMM_SUCCESS) ++fails;
    wr.txPacket();
    auto now = std::chrono::steady_clock::now();
    if (i) dt.push_back(std::chrono::duration<double, std::milli>(now - prev).count());
    prev = now;
  }
  double m = 0; for (double x : dt) m += x; m /= dt.size();
  double s = 0; for (double x : dt) s += (x - m) * (x - m); s = std::sqrt(s / dt.size());
  std::sort(dt.begin(), dt.end());
  printf("%s @ %d: %d cycles, %.1f Hz, %.3f +- %.3f ms, p50 %.3f p99 %.3f max %.3f, failed reads %d\n",
         port + strlen(port) - 30, baud, n, 1000.0 / m, m, s, dt[dt.size() / 2],
         dt[dt.size() * 99 / 100], dt.back(), fails);
}
