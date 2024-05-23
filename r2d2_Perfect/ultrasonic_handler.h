#include "NewPing.h"

namespace ultraSonic {
byte TRIGGER_PIN;
byte ECHO_PIN;
int MAX_DISTANCE = 400;

NewPing* sonar;

float duration, distance;
int iterations = 5;
void setup(byte trig, byte echo) {
  TRIGGER_PIN = trig;
  ECHO_PIN = echo;
  sonar = new NewPing(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
}

float calculateDistance() {
  duration = sonar->ping_median(iterations);
  distance = (duration / 2) * 0.0343;
  return distance;
}
}
