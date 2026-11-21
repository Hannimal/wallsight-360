#pragma once
// Fixed-position room scan: persistent angular surface map and straight-line candidates.
// Pure C++ for desktop tests and Arduino builds (no dynamic memory allocation).
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace roommap {
constexpr int BINS = 360;
constexpr float RAD = 0.01745329251994329577f;
constexpr uint16_t MIN_MM = 180;
constexpr uint16_t MAX_MM = 12000;
constexpr uint8_t MIN_STABLE_HITS = 5;
constexpr uint8_t MIN_LINE_CONF = 5;

struct Segment { float x1, y1, x2, y2; uint16_t lengthMm; };

inline int wrapAngle(int degrees) {
  degrees %= BINS;
  return degrees < 0 ? degrees + BINS : degrees;
}
inline void polarToCartesian(int angle, uint16_t dist, float &x, float &y) {
  float theta = angle * RAD;
  x = dist * sinf(theta); // +X right
  y = dist * cosf(theta); // +Y forwards
}
inline float hypot2(float x, float y) { return sqrtf(x*x + y*y); }

struct PolarMap {
  uint16_t mm[BINS] = {};
  uint8_t confidence[BINS] = {};
  uint16_t candidate[BINS] = {};
  uint8_t hits[BINS] = {};

  void clear() { memset(this, 0, sizeof(*this)); }
  int countStable() const {
    int n = 0;
    for (int i=0; i<BINS; ++i) if (mm[i] && confidence[i]>=MIN_LINE_CONF) ++n;
    return n;
  }
  void observe(int angle, uint16_t d) {
    if (d < MIN_MM || d > MAX_MM) return;
    const int k = wrapAngle(angle);
    if (mm[k] != 0 && absdiff(mm[k], d) <= 260) {
      mm[k] = (uint16_t)((7UL*mm[k] + d + 4) / 8);
      if (confidence[k] < 250) ++confidence[k];
      candidate[k] = 0;
      hits[k] = 0;
      return;
    }
    // New or changed surfaces must be observed repeatedly before updating map.
    if (candidate[k] == 0 || absdiff(candidate[k], d) > 260) {
      candidate[k] = d;
      hits[k] = 1;
    } else {
      candidate[k] = (uint16_t)((3UL*candidate[k] + d + 2) / 4);
      if (hits[k] < 250) ++hits[k];
    }
    // Retain already observed walls unless an alternative persists for much longer.
    const int threshold = mm[k] ? 45 : MIN_STABLE_HITS;
    if (hits[k] >= threshold) {
      mm[k] = candidate[k];
      confidence[k] = MIN_LINE_CONF;
      candidate[k] = 0;
      hits[k] = 0;
    }
  }
  static uint16_t absdiff(uint16_t a, uint16_t b) {
    return a>b ? a-b : b-a;
  }

  // Greedy segmentation over consecutive polar points. These are straight
  // SURFACE candidates, not proof that the object is a structural wall.
  int getSegments(Segment *out, int capacity, float minLenMm=850.f,
                  float maxDeviationMm=160.f, float maxGapMm=600.f) const {
    int found = 0, i = 0;
    while (i < BINS && found < capacity) {
      while (i<BINS && (mm[i]==0 || confidence[i]<MIN_LINE_CONF)) ++i;
      if (i >= BINS) break;
      const int start = i;
      float ax,ay; polarToCartesian(start,mm[start],ax,ay);
      float bx=ax, by=ay;
      int last = start;
      int points = 1;
      int next = start+1;
      for (; next<BINS; ++next) {
        if (!mm[next] || confidence[next]<MIN_LINE_CONF) {
          if (next-last>3) break;
          continue;
        }
        float x,y; polarToCartesian(next,mm[next],x,y);
        if (hypot2(x-bx,y-by)>maxGapMm) break;
        const float len=hypot2(x-ax,y-ay);
        if (len>200.f) {
          bool fits=true;
          for (int j=start+1;j<next;++j) {
            if (!mm[j] || confidence[j]<MIN_LINE_CONF) continue;
            float px,py; polarToCartesian(j,mm[j],px,py);
            const float perp=fabsf((x-ax)*(ay-py)-(ax-px)*(y-ay))/len;
            if (perp>maxDeviationMm) {fits=false;break;}
          }
          if (!fits) break;
        }
        last=next; bx=x;by=y; ++points;
      }
      const float length=hypot2(bx-ax,by-ay);
      if (points>=7 && length>=minLenMm) {
        out[found++] = {ax,ay,bx,by,(uint16_t)length};
        i=last+1;
      } else {
        // Do not skip across a discontinuity; allow the next point to begin a new segment.
        i=(last>start)?last+1:start+1;
      }
    }
    return found;
  }
};

constexpr uint32_t STORAGE_MAGIC = 0x524D5032UL; // 'RMP2'
constexpr uint16_t STORAGE_VERSION = 2;
struct StoredMap {
  uint32_t magic;
  uint16_t version;
  uint16_t binCount;
  uint16_t mm[BINS];
  uint8_t confidence[BINS];
  uint32_t checksum;
};
inline uint32_t checksum(const StoredMap &s) {
  const uint8_t *ptr=(const uint8_t*)&s;
  uint32_t hash=2166136261UL;
  for (size_t i=0;i<sizeof(StoredMap)-sizeof(uint32_t);++i) {
    hash^=ptr[i]; hash*=16777619UL;
  }
  return hash;
}
inline void buildStored(const PolarMap& map, StoredMap& out) {
  memset(&out,0,sizeof(out));
  out.magic=STORAGE_MAGIC;out.version=STORAGE_VERSION;out.binCount=BINS;
  memcpy(out.mm,map.mm,sizeof(out.mm));
  memcpy(out.confidence,map.confidence,sizeof(out.confidence));
  out.checksum=checksum(out);
}
inline bool restoreStored(const StoredMap& s, PolarMap& map) {
  if (s.magic!=STORAGE_MAGIC || s.version!=STORAGE_VERSION ||
      s.binCount!=BINS || checksum(s)!=s.checksum) return false;
  for (int i=0;i<BINS;++i) {
    if (s.mm[i]>MAX_MM || (s.mm[i]!=0 && s.mm[i]<MIN_MM)) return false;
  }
  map.clear();
  memcpy(map.mm,s.mm,sizeof(s.mm));
  memcpy(map.confidence,s.confidence,sizeof(s.confidence));
  return true;
}
} // namespace roommap
