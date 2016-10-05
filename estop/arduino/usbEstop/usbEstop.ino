// USB E-stop button for Maker Faire Berlin 2016
// Copyright (c) 2016 Bitcraze AB
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Based on example by Matthew Heironimus
// 2015-11-20
//--------------------------------------------------------------------

#include <Joystick.h>

// Pin where the button is connected
const int buttonPin = 9;

void setup() {
  // Initialize Button Pin
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize Joystick Library
  Joystick.begin();
}

// Last state of the button
int lastButtonState = 0;

void loop() {

  // Read pin value
  int currentButtonState = !digitalRead(buttonPin);
  if (currentButtonState != lastButtonState)
  {
    Joystick.setButton(0, !currentButtonState);
    lastButtonState = currentButtonState;
  }

  delay(50);
}
