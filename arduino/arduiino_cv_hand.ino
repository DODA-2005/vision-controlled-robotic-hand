
// five_servos_plotter.ino
// Receives: "T,I,M,R,P\n"  (angles 0-180)
// Thumb -> D4, Index -> D5, Middle -> D6, Ring -> D7, Pinky -> D8
// Continuously prints current angles as 5 numbers per line for Serial Plotter.

#include <Servo.h>

Servo sThumb, sIndex, sMiddle, sRing, sPinky;
const int pinThumb  = 4;
const int pinIndex  = 5;
const int pinMiddle = 6;
const int pinRing   = 7;
const int pinPinky  = 8;

unsigned long lastPlot = 0;
const unsigned long PLOT_INTERVAL_MS = 100; // 10 Hz plotting

int currentAngles[5] = {0,0,0,0,0};

String readLine(unsigned long timeout_ms) {
  unsigned long start = millis();
  String s = "";
  while (millis() - start < timeout_ms) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') return s;
      if (c >= 32 || c == '\r' || c == '\t') s += c;
    }
  }
  return s;
}

void applyAngle(Servo &sv, int a) {
  sv.write(constrain(a, 0, 180));
}

void printAnglesForPlotter() {
  // Print five numbers separated by spaces (Serial Plotter accepts this)
  Serial.print(currentAngles[0]); Serial.print(' ');
  Serial.print(currentAngles[1]); Serial.print(' ');
  Serial.print(currentAngles[2]); Serial.print(' ');
  Serial.print(currentAngles[3]); Serial.print(' ');
  Serial.println(currentAngles[4]);
}

void setup() {
  Serial.begin(9600); // keep at 9600 (matches Python). You may use 115200 if you update Python.
  sThumb.attach(pinThumb);
  sIndex.attach(pinIndex);
  sMiddle.attach(pinMiddle);
  sRing.attach(pinRing);
  sPinky.attach(pinPinky);

  // start open (0 deg)
  currentAngles[0] = 0;
  currentAngles[1] = 0;
  currentAngles[2] = 0;
  currentAngles[3] = 13; // ring open default 13 as you wanted opening servo value
  currentAngles[4] = 0;

  applyAngle(sThumb,  currentAngles[0]);
  applyAngle(sIndex,  currentAngles[1]);
  applyAngle(sMiddle, currentAngles[2]);
  applyAngle(sRing,   currentAngles[3]);
  applyAngle(sPinky,  currentAngles[4]);

  delay(200);
  Serial.println("READY"); // this line will be ignored by the plotter because plotting waits on numeric lines
  lastPlot = millis();
}

void loop() {
  // Non-blocking read of one incoming line (short timeout)
  if (Serial.available()) {
    String line = readLine(50);
    line.trim();
    if (line.length() > 0) {
      // parse CSV "T,I,M,R,P"
      int vals[5] = {0,0,0,0,0};
      int idx = 0;
      char buf[128];
      line.toCharArray(buf, sizeof(buf));
      char *tok = strtok(buf, ",");
      while (tok != NULL && idx < 5) {
        vals[idx++] = atoi(tok);
        tok = strtok(NULL, ",");
      }
      // update currentAngles and servos for those we parsed
      for (int i = 0; i < 5; ++i) {
        currentAngles[i] = constrain(vals[i], 0, 180);
      }
      applyAngle(sThumb,  currentAngles[0]);
      applyAngle(sIndex,  currentAngles[1]);
      applyAngle(sMiddle, currentAngles[2]);
      applyAngle(sRing,   currentAngles[3]);
      applyAngle(sPinky,  currentAngles[4]);

      // optional: immediately print a numeric line for plotter after new command
      printAnglesForPlotter();
      lastPlot = millis();
    }
  }

  // Periodically print numeric line for plotter so it's continuous
  if (millis() - lastPlot >= PLOT_INTERVAL_MS) {
    printAnglesForPlotter();
    lastPlot = millis();
  }
}