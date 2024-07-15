#ifndef _PERFORMANCE_
#define _PERFORMANCE_

#include "Arduino.h"

struct Profile
{
  char *name;
  unsigned long start;
  unsigned long end;
};

struct Profiles
{
  Profile robotLoop = {"Robot Loop", NULL, NULL};
  Profile mode0 = {"\tMode 0", NULL, NULL};
  Profile modeOther = {"\tMode 1/2/3", NULL, NULL};
  Profile loopTiming = {"\tLoop Timing", NULL, NULL};
  Profile kinematics = {"\tKinematics", NULL, NULL};
  Profile updateMotorSpeeds = {"\tUpdate Motor Speeds", NULL, NULL};
  Profile getImuZForVest = {"\tGet IMU Z For Vest", NULL, NULL};
  Profile outerLoop = {"\tOuter Loop", NULL, NULL};
  Profile centerVestAngle = {"\tCenter Vest Angle", NULL, NULL};

};

double getTime(Profile &profile);
void printProfile(Profile &profile);
void printProfiles(Profiles &profiles);
void startProfile(Profile &profile);
void endProfile(Profile &profile);

#endif
