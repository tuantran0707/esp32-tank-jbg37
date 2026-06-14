#pragma once

#include <Arduino.h>
#include "config.h"

enum Heading { HEAD_NORTH = 0, HEAD_EAST = 1, HEAD_SOUTH = 2, HEAD_WEST = 3 };

struct Pose {
  int x;
  int y;
  Heading heading;
};

// Encoder đếm bằng ngắt (giống code xe cân bằng) — không cần ESP32Encoder
class Odometry {
public:
  Odometry();

  void begin();
  void resetCounts();
  long getRightCount() const;
  long getLeftCount() const;
  long getAvgCount() const;

  void setCountsPerCm(float v) { countsPerCm_ = v; }
  void setTurn90Counts(long v) { turn90Counts_ = v; }
  float countsPerCm() const { return countsPerCm_; }
  long turn90Counts() const { return turn90Counts_; }
  long countsForCm(int cm) const;

private:
  float countsPerCm_;
  long turn90Counts_;
};
