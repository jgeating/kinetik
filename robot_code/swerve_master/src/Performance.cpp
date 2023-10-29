#include "Performance.h"
#include "Swerve.h"

double getTime(Profile &profile) {
  if  (profile.start == NULL || profile.end == NULL) {
    return 0;
  }
  return (profile.end - profile.start) / 1000.0;
}

void printProfile(Profile &profile)
{
  if (profile.start != NULL && profile.end != NULL)
  {
    double time = getTime(profile);
    uint frequency = 1000 / time;
    serialPrintln(100, "%s: %3.3lf ms / %3u Hz", profile.name, time, frequency);
  }

  profile.start = NULL;
  profile.end = NULL;
}


void printProfiles(Profiles &profiles)
{
  serialPrintln(100, "Timing:\n");

  if (getTime(profiles.kinematics) > getTime(profiles.robotLoop)) {
    serialPrintln(100, "RobotLoop start: %lu:\n", profiles.robotLoop.start);
    serialPrintln(100, "RobotLoop end: %lu:\n", profiles.robotLoop.end);
    serialPrintln(100, "Kinematics start: %lu:\n", profiles.kinematics.start);
    serialPrintln(100, "Kinematics end: %lu:\n", profiles.kinematics.end);
  }

  double time = (
    getTime(profiles.mode0) 
    + getTime(profiles.modeOther) 
    + getTime(profiles.loopTiming) 
    + getTime(profiles.kinematics) 
    + getTime(profiles.updateMotorSpeeds) 
    + getTime(profiles.getImuZForVest) 
    + getTime(profiles.outerLoop) 
    + getTime(profiles.centerVestAngle) 
  ); 
  
  printProfile(profiles.robotLoop);
  printProfile(profiles.mode0);
  printProfile(profiles.modeOther);
  printProfile(profiles.loopTiming);
  printProfile(profiles.kinematics);
  printProfile(profiles.updateMotorSpeeds);
  printProfile(profiles.getImuZForVest);
  printProfile(profiles.outerLoop);
  printProfile(profiles.centerVestAngle);
  serialPrintln(100, "\tTime: %3.3lf ms", time);
  Serial.println("\n");
}

void startProfile(Profile &profile)
{
  profile.start = micros();
  // serialPrintln(100, "%s start: %lu:\n", profile.name, profile.start);
}

void endProfile(Profile &profile)
{
  profile.end = micros();
  // serialPrintln(100, "%s end: %lu:\n", profile.name, profile.end);
}
