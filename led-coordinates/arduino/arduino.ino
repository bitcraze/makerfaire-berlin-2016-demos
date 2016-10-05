
/*
# The MIT License (MIT)
# Copyright (c) 2016 Bitcraze AB
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
*/




/*
 *  Arduino code used to display positions on 3 LED strips fixed to the X, Y
 *  Z axis of the stand.
 *  Positions are fed from ROS/python over a serial port.
 *
 *  Based on Adafruits NeoPixel library.
 */


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN_XY 6

#define PIXELS_PER_STRIP 150
#define PIXELS_PER_STRIP_SHORT 126
#define LEDS_PER_METER 30.0

#define SYNC 0xbc


#define X_MIN_LED 0
#define X_MAX_LED 1
#define MAX_Y 2.0
#define X_ZERO_PIXEL 2 * PIXELS / 3
#define Y_ZERO_PIXEL X_ZERO_PIXEL + 1

uint32_t col = 0;
int xPrevPixel = 0;
int yPrevPixel = 0;

float x = 0.0;
float y = 0.0;
float z = 0.0;
int pos = 0;

#define MESSAGE_LEN 8
int byteInCount = 0;
unsigned int inData[MESSAGE_LEN];

typedef struct {
  uint32_t color;
  float x, y, z;
} Position;

#define NR_OF_POSITIONS 3
Position position[NR_OF_POSITIONS] = {
  {.color=0x00ffffff, .x = -10.0, .y = -10.0, .z = -10.0},
  {.color=0x00ff0000, .x = -10.0, .y = -10.0, .z = -10.0},
  {.color=0x0000ff00, .x = -10.0, .y = -10.0, .z = -10.0}
};

Adafruit_NeoPixel stripXY = Adafruit_NeoPixel(276, PIN_XY, NEO_GRB + NEO_KHZ800);

typedef struct {
  float multiplier;         // Multiplier to convert from distance to led. Usually LEDS_PER_METER or -LEDS_PER_METER
  int zeroLed;              // The led in the strip that is origo for this axiz
  int minLed;               // The first led of this axis 
  int maxLed;               // The last led in this axis
  Adafruit_NeoPixel* strip; // The strip that this axis is rendered on 
} Axis;

Axis xAxis = {
  .multiplier = -LEDS_PER_METER,
  .zeroLed = 215,
  .minLed = 155,
  .maxLed = 215,
  .strip = &stripXY
};

Axis yAxis = {
  .multiplier = LEDS_PER_METER,
  .zeroLed = 215,
  .minLed = 216,
  .maxLed = 276,
  .strip = &stripXY
};

Axis zAxis = {
  .multiplier = LEDS_PER_METER,
  .zeroLed = 86,
  .minLed = 86,
  .maxLed = 151,
  .strip = &stripXY
};


void setup() {
  Serial.begin(115200);
  
  stripXY.begin();
  stripXY.show(); // Initialize all pixels to 'off'

  #if 0
  // Calibration
  setPosition(0, 0.0, 0.0, 0.0);
  setPosition(1, 1.0, 1.0, 1.0);
  setPosition(2, 1.8, 1.8, 1.8);
  redraw();
  #endif
}

void loop() {
  if (Serial.available() > 0) {
    bool isCompleteMessage = handleInput(Serial.read());
    if (isCompleteMessage) {
      setPosition(pos, x, y, z);
      redraw();
    }
  }
}



/*
 * Input format
 * 0xbc SYNC 8 bits
 * pos index 8 bits
 * X 16 bits
 * Y 16 bits
 * Z 16 bits
 */
bool handleInput(unsigned char data) {
  bool result = false;
  inData[byteInCount] = data;
  
  if (0 == byteInCount) {
    if (SYNC == data) {
      byteInCount = 1;    
    }
  } else {    
    byteInCount++;

    if (MESSAGE_LEN == byteInCount) {
      byteInCount = 0;

      pos = inData[1];
      x = float(((int)inData[2] << 8) + inData[3]) / 100.0;
      y = float(((int)inData[4] << 8) + inData[5]) / 100.0;
      z = float(((int)inData[6] << 8) + inData[7]) / 100.0;
      result = true;
    }
  }
  
  return result;
}

void setPosition(uint8_t pos, float x, float y, float z) {
  if (pos < NR_OF_POSITIONS) {
    position[pos].x = x;
    position[pos].y = y;
    position[pos].z = z;
  }
}


void redraw() {
  stripXY.clear();  

  for (int i = 0; i < NR_OF_POSITIONS; i++) {
    render(&position[i]);    
  }

  stripXY.show();
}

void render(Position* position) {
  uint32_t color = position->color;
  renderAxis(&xAxis, position->x, color);
  renderAxis(&yAxis, position->y, color);
  renderAxis(&zAxis, position->z, color);
}

void renderAxis(Axis* axis, float val, uint32_t color) {
  int led = axis->zeroLed + round(val * axis->multiplier); 
  if (led >= axis->minLed && led <= axis->maxLed) {
    axis->strip->setPixelColor(led, color);
  }
}

