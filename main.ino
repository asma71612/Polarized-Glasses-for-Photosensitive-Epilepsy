#include "arduinoFFT.h"
#include <Servo.h>

#define SAMPLES 128            //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 512 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
 
arduinoFFT FFT = arduinoFFT();
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int servoPin = 3;
 
unsigned int samplingPeriod;
unsigned long microSeconds;
 
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
 
void setup() 
{
    Serial.begin(115200); //Baud rate for the Serial Monitor
    samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds 
    myservo.attach(servoPin);

}
 
void loop() 
{  
    int frequency = findFrequency(0); //Find frequency using the signal on analog pin 0 on the Arduino
    Serial.println(frequency);
    int rotationAmountx = rotationAmount(frequency); //Figure out by how much to rotate the motor
    rotateMotorFunction(rotationAmountx, 2); //Rotate the motor
}

void rotateMotorFunction (int endPos,int delayNum) {
  for (int startPos = 0; startPos <= endPos; startPos += 1) { // goes from 0 degrees to 120 degrees
    // in steps of 1 degree
    myservo.write(startPos);              // tell servo to go to position in variable 'pos'
    delay(delayNum);                       // waits 15ms for the servo to reach the position
  }
  
  int currentFrequency = findFrequency(0);
  while(endPos/1.4 + 5 > currentFrequency && endPos/1.4 - 5 < currentFrequency){
    currentFrequency = findFrequency(0);
    //While the light frequency is within 5Hz of the initial frequency, do not rotate the motor back
  }
  
   for (int goingBack = endPos; goingBack >= 0; goingBack -= 1) { // goes from 120 degrees to 0 degrees
    myservo.write(goingBack);              // tell servo to go to position in variable 'pos'
    delay(delayNum);                       // waits 15ms for the servo to reach the position
  }
}

double findFrequency(int phototransistorAnalogPinNumber) { 
  /*Sample SAMPLES times*/
    for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
     
        vReal[i] = analogRead(phototransistorAnalogPinNumber); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
        vImag[i] = 0; //Makes imaginary term 0 always

        /*remaining wait time between samples if necessary*/
        while(micros() < (microSeconds + samplingPeriod))
        {
          //do nothing
        }
    }
   
 
    /*Perform FFT on samples*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    /*Find peak frequency and print peak*/
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    return peak;
}

int rotationAmount(int rawFrequency) {
  int frequency = round(rawFrequency);
  if (frequency < 10 || frequency > 50) return 0;
  return frequency * 1.4;
}
