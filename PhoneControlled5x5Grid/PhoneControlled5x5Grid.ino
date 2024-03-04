//Developed by S.A.M. Weima for the "Programmable actuation of liquid crystal fibers"

const int ledPin = LED_BUILTIN;  // the number of the LED pin

int ledState = LOW;  // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated
long OnTime = 50;                  // milliseconds of on-time
long OffTime = 950;                // milliseconds of off-time

const int buttons[4] = {
  2, 4, 7, 8
};

const int actuators[4] = {
  3, 5, 6, 9
};

byte intensity = 0;

int activeResistor = 1;

int program = -1;

int pulseCount = 0;

int pulseMax = 20;

int buttonPress = -1;
// PID
#include <PID_v2.h>

// Specify the links and initial tuning parameters
double Kp = 0.4, Ki = 0.25, Kd = 0.2;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

// input output and setpoint without capitals!
double input = 0;   // input temperature measured by the thermistor
double output = 0;  // output variable, here the output will be dumped (a number between and including 0 and 255)

double setpoint = 70;

const int thermistors[4] = {
  14, 15, 16, 17
};

float Vo[4], temp[4];
;

// From thermistor calibration
float thermistorPolynomial[4][4] = {
  { -2.02506E-06, 0.00460753, -3.61149, 1058.9 },
  { -2.19803E-06, 0.00510579, -4.07767, 1203.71 },
  { -1.93726E-06, 0.00442195, -3.47802, 1026.78 },
  { -1.9299E-06, 0.00441881, -3.48918, 1033.78 }
};


int averageNsamples = 500;
int voltageSample = 0;

// For phone control
int phoneControlled = 0;
int directionDeg = 0;
int heatTime = 10;
int coolTime = 1000;
int heatState = 1;
int direction = 0;
void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);

  for (int thisPin = 0; thisPin < 4; thisPin++) {
    // initialize the actuator pins:
    pinMode(actuators[thisPin], OUTPUT);
    analogWrite(thisPin, 0);
  }

  for (int thisPin = 0; thisPin < 4; thisPin++) {
    // initialize the button input pins:
    pinMode(buttons[thisPin], INPUT_PULLUP);
  }

  for (int thisPin = 0; thisPin < 4; thisPin++) {
    // initialize the thermistor input pins:
    pinMode(thermistors[thisPin], INPUT);
  }

  myPID.Start(input, output, setpoint);

  // if you want to change the setpoint only
  myPID.Setpoint(setpoint);

  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Native USB only
  }
}

void loop() {
  if (Serial.available()) {
    phoneControlled = 1;
    reset();

    String data = Serial.readString();
    data.trim();

    if (data.charAt(0) == 'a') {
      data.remove(0, 1);
      directionDeg = data.toInt();
    } else if (data.charAt(0) == 'h') {
      data.remove(0, 1);
      heatTime = data.toInt()/10;
    }
    // directionDeg = Serial.parseInt();
    Serial.println(directionDeg);
  }
  if (phoneControlled) {
    phoneControl();
  }
  if (!phoneControlled) {
    checkButtons();

    if (program == -1) {
      reset();
    }

    if (buttonPress == -1) {
      if (program == 0) {
        program0();
      } else if (program == 1) {
        program1();
      } else if (program == 2) {
        program2();
      } else if (program == 3) {
        program3();
      }
    }
  }
}
void phoneControl() {
  
  if (directionDeg < 23 || directionDeg >= 338) {
    direction = 0;
  } else if (directionDeg >= 23 && directionDeg < 68) {
    direction = 4;
  } else if (directionDeg >= 68 && directionDeg < 113) {
    direction = 1;
  } else if (directionDeg >= 113 && directionDeg < 158) {
    direction = 5;
  } else if (directionDeg >= 158 && directionDeg < 203) {
    direction = 2;
  } else if (directionDeg >= 203 && directionDeg < 248) {
    direction = 6;
  } else if (directionDeg >= 248 && directionDeg < 293) {
    direction = 3;
  } else if (directionDeg >= 293 && directionDeg < 338) {
    direction = 7;
  }
  if (direction <= 3) {
    if ((heatState == 1) && (millis() - previousMillis >= heatTime)) {
      previousMillis = millis();
      heatState = 0;
      reset();
    } else if ((heatState == 0) && (millis() - previousMillis >= coolTime)) {
      previousMillis = millis();
      heatState = 1;
      analogWrite(actuators[direction], 255);
      Serial.println("ON");
    }
  } else {
    int localHeatTime = heatTime / 2;  // function to adjust heatTime for diagonal directions, to keep overall power constant and prevent overheating
    if ((heatState == 1) && (millis() - previousMillis >= localHeatTime)) {
      previousMillis = millis();
      heatState = 0;
      reset();
    } else if ((heatState == 0) && (millis() - previousMillis >= coolTime)) {
      previousMillis = millis();
      heatState = 1;
      if (direction == 7) {
        analogWrite(actuators[3], 255);
        analogWrite(actuators[0], 255);
      } else {
        analogWrite(actuators[direction - 4], 255);
        analogWrite(actuators[direction - 3], 255);
      }
      Serial.println("ON");
    }
  }
}
void program0() {
  intensity = 255;

  unsigned long currentMillis = millis();

  if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
    ledState = LOW;                  // Turn it off
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    reset();

  } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
    ledState = HIGH;                 // turn it on
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    analogWrite(actuators[0], intensity);
  }
}

void program1() {
  //pulses per resistor
  pulseMax = 1;
  intensity = 255;

  unsigned long currentMillis = millis();

  if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
    ledState = LOW;                  // Turn it off
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    reset();
  } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
    ledState = HIGH;                 // turn it on
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    reset();

    if ((activeResistor == 3) && (pulseCount == pulseMax)) {
      activeResistor = 0;
      pulseCount = 0;
    } else if (pulseCount == pulseMax) {
      activeResistor = activeResistor + 1;
      pulseCount = 0;
    }

    for (int thisPin = 0; thisPin < 4; thisPin++) {
      if (activeResistor == thisPin) {
        analogWrite(actuators[thisPin], intensity);
      }
    }
    pulseCount = pulseCount + 1;
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
void program2() {
  //Alternating
  pulseMax = 1;
  intensity = 255;
  unsigned long currentMillis = millis();

  if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime)) {
    ledState = LOW;                  // Turn it off
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    reset();
  } else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime)) {
    ledState = HIGH;                 // turn it on
    previousMillis = currentMillis;  // Remember the time
    digitalWrite(ledPin, ledState);  // Update the actual LED

    reset();

    //Preventing dead end
    if (activeResistor == 0) {
      activeResistor = 1;
    }
    if (activeResistor == 2) {
      activeResistor = 1;
    }

    if ((activeResistor == 3) && (pulseCount == pulseMax)) {
      activeResistor = 1;
      pulseCount = 0;
    } else if (pulseCount == pulseMax) {
      activeResistor = 3;
      pulseCount = 0;
    }
    for (int thisPin = 0; thisPin < 4; thisPin++) {
      if (activeResistor == thisPin) {
        analogWrite(actuators[thisPin], intensity);
      }
    }
    pulseCount = pulseCount + 1;
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
void program3() {

  myPID.Setpoint(setpoint);
  readThermistors();

  //Uniform heating for fiber fabrication.
  output = myPID.Run(input);
  intensity = output / 255 * 100;

  for (int thisPin = 0; thisPin < 4; thisPin++) {
    analogWrite(actuators[thisPin], intensity);
  }
}

void readThermistors() {

  if (voltageSample < averageNsamples) {
    voltageSample += 1;
    Vo[0] += analogRead(thermistors[0]);
    Vo[1] += analogRead(thermistors[1]);
    Vo[2] += analogRead(thermistors[2]);
    Vo[3] += analogRead(thermistors[3]);
  } else {
    voltageSample = 0;
    Vo[0] = analogRead(thermistors[0]);
    Vo[1] = analogRead(thermistors[1]);
    Vo[2] = analogRead(thermistors[2]);
    Vo[3] = analogRead(thermistors[3]);
  }
  if (voltageSample == averageNsamples) {
    for (int thisThermistor = 0; thisThermistor < 4; thisThermistor++) {
      Vo[thisThermistor] /= averageNsamples + 1;  //Take an average to minimize the effect of noise
      temp[thisThermistor] = thermistorPolynomial[thisThermistor][3] + thermistorPolynomial[thisThermistor][2] * Vo[thisThermistor] + thermistorPolynomial[thisThermistor][1] * Vo[thisThermistor] * Vo[thisThermistor] + thermistorPolynomial[thisThermistor][0] * Vo[thisThermistor] * Vo[thisThermistor] * Vo[thisThermistor];
    }
    Serial.print("temp1:");
    Serial.print(temp[0]);
    Serial.print(", ");
    Serial.print("temp2:");
    Serial.print(temp[1]);
    Serial.print(", ");
    Serial.print("temp3:");
    Serial.print(temp[2]);
    Serial.print(", ");
    Serial.print("temp4:");
    Serial.print(temp[3]);
    Serial.print(", ");
    Serial.print("Vo1:");
    Serial.print(Vo[0]);
    Serial.print(", ");
    Serial.print("Vo2:");
    Serial.print(Vo[1]);
    Serial.print(", ");
    Serial.print("Vo3:");
    Serial.print(Vo[2]);
    Serial.print(", ");
    Serial.print("Vo4:");
    Serial.print(Vo[3]);
    Serial.print(", ");
    Serial.print("input:");
    Serial.print(input);
    Serial.print(", ");
    Serial.print("intensity:");
    Serial.println(intensity);
  }
  //PID: enter new temperature into PID libary loop
  input = (temp[0] + temp[1] + temp[2] + temp[3]) / 4;  //average over thermistors close to traces
}

void checkButtons() {
  for (int thisPin = 0; thisPin < 4; thisPin++) {
    if (digitalRead(buttons[thisPin]) == 0) {
      reset();
      buttonPress = thisPin;
      delay(2);
    }
    if (digitalRead(buttons[thisPin]) == 1) {
      if (buttonPress == thisPin) {
        buttonPress = -1;
        if (program == -1) {
          program = thisPin;
        } else if (program == thisPin) {
          program = -1;
        }
      }
    }
  }
}

void reset() {
  for (int resetPin = 0; resetPin < 4; resetPin++) {
    // turn off the actuator pins:
    analogWrite(actuators[resetPin], 0);
  }
}