#include <cassert>
#include <cstdio>
#include <cmath>
#include "../include/wall_map.h"
using namespace roommap;

// Perfect rectangular room: x in [-2.5, 2.5] m, y in [-3.5, 3.5] m
static uint16_t rangeInRoom(int angle) {
  float x=sinf(angle*RAD), y=cosf(angle*RAD);
  float result=100000.f;
  if (fabsf(x)>0.001f) result=fminf(result,2500.f/fabsf(x));
  if (fabsf(y)>0.001f) result=fminf(result,3500.f/fabsf(y));
  return (uint16_t)lroundf(result);
}
int main() {
  PolarMap map;
  assert(map.countStable()==0);
  for (int round=0;round<12;++round)
    for (int angle=0;angle<360;++angle)
      map.observe(angle,rangeInRoom(angle)+(round%3-1)*15);
  assert(map.countStable()==360);
  Segment walls[30];
  const int lines=map.getSegments(walls,30);
  printf("Stable bins: %d / 360, straight surface segments: %d\n",map.countStable(),lines);
  assert(lines>=3);
  uint16_t before=map.mm[20];
  // A passing target affects a few observations, but cannot immediately replace the wall.
  for (int i=0;i<4;++i) map.observe(20,1200);
  assert(map.mm[20]==before);
  StoredMap snapshot;
  buildStored(map,snapshot);
  PolarMap restored;
  assert(restoreStored(snapshot,restored));
  assert(restored.mm[20]==before && restored.countStable()==360);
  snapshot.mm[20]^=0x01;
  assert(!restoreStored(snapshot,restored));
  printf("Persistence checksum & transient rejection: PASS\n");
}
