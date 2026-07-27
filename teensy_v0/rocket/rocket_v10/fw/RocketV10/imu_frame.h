#pragma once

// ST LSM9DS1 datasheet Figure 1: magnetic X/Y directions are opposite
// accelerometer/gyro X/Y, while Z is shared. The Adafruit driver returns each
// die's native register axes without making this 180-degree Z remap.
inline void lsm9ds1MagToAgFrame(float calibratedMxUt, float calibratedMyUt,
                                float calibratedMzUt,
                                float &bodyMxUt, float &bodyMyUt,
                                float &bodyMzUt) {
  bodyMxUt = -calibratedMxUt;
  bodyMyUt = -calibratedMyUt;
  bodyMzUt = calibratedMzUt;
}
