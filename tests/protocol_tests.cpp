#include <cassert>
#include <cstdio>
#include "../include/radar_protocol.h"
int main() {
  using namespace radarproto;
  // Manufacturer LD2450 example: x=-782 mm, y=+1713 mm, v=-16 cm/s.
  uint8_t radar[30] = {0xAA,0xFF,0x03,0x00,
     0x0E,0x03,0xB1,0x86,0x10,0x00,0x40,0x01,
     0,0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,0,
     0x55,0xCC};
  Target ts[3] = {};
  assert(decodeLD2450(radar,30,ts));
  assert(ts[0].valid && ts[0].xMm==-782 && ts[0].yMm==1713);
  assert(ts[0].speedCmS == -16 && ts[0].resolutionMm == 320);
  assert(!ts[1].valid && !ts[2].valid);
  radar[29]=0;
  assert(!decodeLD2450(radar,30,ts));
  // Synthetic LD19 47-byte scan; self-consistent CRC + angle structure.
  uint8_t lidar[47] = {0x54,0x2C};
  lidar[2]=0x10; lidar[3]=0x0E; // 3600 degrees/s
  lidar[4]=0x00; lidar[5]=0x00; // 0 degree
  for (int i=0;i<12;i++) { lidar[6+i*3]=0xE8; lidar[7+i*3]=0x03; lidar[8+i*3]=0xF0; }
  lidar[42]=0x10; lidar[43]=0x27; // 100 degrees
  lidar[46]=ld19Crc(lidar,46);
  assert(validLD19(lidar,47));
  lidar[6]^=0x01;
  assert(!validLD19(lidar,47));
  puts("LD19 CRC and LD2450 frame tests: PASS");
}
