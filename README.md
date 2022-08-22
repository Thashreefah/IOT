# IOT

PROGRAM-1.0(Arduino LED)<br>
https://wokwi.com/projects/333796636268429907

PROGRAM-1(Arduino 3-LED)<br>
https://wokwi.com/projects/333716380233761363

PROGRAM-2(Arduino RGB LED)<br>
https://wokwi.com/projects/333885849725829714

PROGRAM-3(Arduino LCD)<br>
https://wokwi.com/projects/334974475061690963

🤍SERVO MOTOR🤍

PROGRAM-4 (SERVO MOTOR)<br>
https://wokwi.com/projects/334977337093259858

PROGRAM-5 (SERVOMOTOR + POTENTIOMETER SLIDE)<br>
https://wokwi.com/projects/334980377426788946

PROGRAM-6 (SERVOMOTOR + PUSHBUTTON)<br>
https://wokwi.com/projects/335702709745222226

PROGRAM-7 (SERVO MOTOR USING 2 FOR LOOPS[BACKWARD AND FORWARD])<BR>
 https://wokwi.com/projects/334981476631183956

🤍BUZZER🤍

 PROGRAM-8 (BUZZER)<br>
 https://wokwi.com/projects/335065040663085650
 
PROGRAM-9(BUZZER + PUSH BUTTON)<br>
https://wokwi.com/projects/335067381555528276

 PROGRAM-9.0(BUZZER + PUSH BUTTON + LED)<br>
 https://wokwi.com/projects/335616285217915474
 
 PROGRAM-10(BUZZER + PUSH BUTTON [LONG PRESS])<br>
 https://wokwi.com/projects/335069571375432274

🤍ULTRASONIC SENSOR🤍

 PROGRAM-11(ULTRASONIC SENSOR)<br>
 https://wokwi.com/projects/335070257026695763
 
 PROGRAM-12(ULTRASONIC SENSOR + BUZZER)<br>
 https://wokwi.com/projects/335072488952169043
 
 PROGRAM-13(ULTRASONIC SENSOR + BUZZER + LED)<br>
 https://wokwi.com/projects/335610155442897491
 
 PROGRAM-14 (2 ULTRASONIC SENSOR + LED + BUZZER)<br>
 https://wokwi.com/projects/335706785117635155
 
 🤍POTENTIOMETER🤍
 
PROGRAM-15 (POTENTIOMETER + LED)<br>
https://wokwi.com/projects/335702361099993682

 
 🤍DHT22🤍
 
 PROGRAM-16 (DHT22 [HUMIDITY AND TEMPERATURE SENSOR])<br>
 https://wokwi.com/projects/338145784550654546
 
 PROGRAM-17 (DHT22 [HUMIDITY AND TEMPERATURE SENSOR] + LCD)<br>
 https://wokwi.com/projects/338146187020337746
 
 🤍ESP32 🤍
 
 PROGRAM-1.0 (ESP32 LED)<br>
 https://wokwi.com/projects/336878768089989716

 PROGRAM-1 (ESP32 3-LED)<br>
https://wokwi.com/projects/336881601441956435

PROGRAM-2 (ESP32 RGB LED)<br>
https://wokwi.com/projects/336881955089941074


**************************************************************************
1. To interface LED/Buzzer with Arduino and write a program to turn ON LED for 1 sec after every 2 seconds.<br>
     https://wokwi.com/projects/334348041107538516 <br>
2. To interface Push button/Digital sensor(IR/LDR) with Arduino and write a program to turn ON LED when push button is pressed or at sensor detection.  <br>
      https://wokwi.com/projects/338224466542723668<br>
3. To interface DHT11(DHT22) sensor with Arduino and write a program to print temperature and humidity readings.<br>
      https://wokwi.com/projects/338145995428725330 <br>
4. To interface motor using relay with Arduino and write a program to turn ON motor when push button is pressed.<br>
      https://wokwi.com/projects/335701119415091795 <br>
5. To interface LCD with Arduino and write a program to print temperature and humidity reading on it.<br>
      https://wokwi.com/projects/338146515814974035 <br>
6. To interface Bluetooth with Arduion and write a program to send sensor data to smartphone using Bluetooth.<br>
7. To interface Bluetooth with Arduion and write a program to turn LED ON/OFF when '1'/'0' is received from smartphone using Bluetooth.<br>
******************************************************************************

❤HARDWARE❤<br>
 
😎BUZZER😎<br>

const int buzzer=2;//D4buzzer to arduino pin 9<br>
void setup() {<br>
  // put your setup code here, to run once:<br>
pinMode(buzzer,OUTPUT);// Set buzzer - pin 9 as an output<br>
}<br>

void loop() {<br>
  // put your main code here, to run repeatedly:<br>
tone(buzzer,10);// Send 1KHz sound signal...<br>
delay(1000);// ...for 10 sec<br>
noTone(buzzer);// Stop sound...<br>
delay(1000);// ...for 5 sec<br>
}<br>

😎RGB😎<br>

int red = D5;<br>
 int green = D6;<br>
 int blue = D7;<br>
 //GROUND IS CONNECTED TO 3V <br>
 void setup() {<br>
   pinMode(red, OUTPUT);<br>
   pinMode(green, OUTPUT);<br>
   pinMode(blue, OUTPUT);<br>

 }<br>

 void loop() {<br>
   displayColor(0b100); //RED<br>
   delay(1000);<br>
   displayColor(0b010); //GREEN<br>
   delay(1000);<br>
   displayColor(0b001); //BLUE<br>
   delay(1000);<br>
   displayColor(0b101); //MAGENTA<br>
   delay(1000);<br>
   displayColor(0b011); //CYAN<br>
   delay(1000);<br>
   displayColor(0b110); //YELLOW<br>
   delay(1000);<br>
   displayColor(0b111); //WHITE<br>
   delay(1000);<br>
 }<br>

 void displayColor(byte color) {<br>
   digitalWrite(red, !bitRead(color, 2));<br>
   digitalWrite(green, !bitRead(color, 1));<br>
   digitalWrite(blue, !bitRead(color, 0));<br>
 }<br>
 
 
 😎ULTRASONIC SENSOR😎<br>
 
 
const int trigPin = 12;//d6<br>
const int echoPin = 14;//d5<br>


//define sound velocity in cm/uS<br>
#define SOUND_VELOCITY 0.034<br>
#define CM_TO_INCH 0.393701<br>

long duration;<br>
float distanceCm;<br>
float distanceInch;<br>
int LED = D0;<br>

void setup() {<br>
  Serial.begin(9600); // Starts the serial communication<br>
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output<br>
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input<br>

  pinMode(LED,OUTPUT);<br>
}<br>

void loop() {<br>
  // Clears the trigPin<br>
  digitalWrite(trigPin, LOW);<br>
  delayMicroseconds(2);<br>
  // Sets the trigPin on HIGH state for 10 micro seconds<br>
  digitalWrite(trigPin, HIGH);<br>
  delayMicroseconds(10);<br>
  digitalWrite(trigPin, LOW);<br>
  
  // Reads the echoPin, returns the sound wave travel time in microseconds<br>
  duration = pulseIn(echoPin, HIGH);<br>
  
  // Calculate the distance<br>
  distanceCm = duration * SOUND_VELOCITY/2;<br>
  
  // Convert to inches<br>
  distanceInch = distanceCm * CM_TO_INCH;<br>
  if(distanceInch < 10)<br>

  digitalWrite(LED,HIGH);<br>
  else<br>
  digitalWrite(LED,LOW); <br>
  
  // Prints the distance on the Serial Monitor<br>
  Serial.print("Distance (cm): ");<br>
  Serial.println(distanceCm);<br>
  Serial.print("Distance (inch): ");<br>
  Serial.println(distanceInch);<br>
  
  delay(1000);<br>
}<br>


😎DHT11😎<br>

 #include <Adafruit_Sensor.h><br>
#include <DHT.h>;<br>
#define DHTPIN D5 // what pin we're connected to<br>
#define DHTTYPE DHT11 // DHT 22 (AM2302)<br>
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino<br>
//Variables<br>
int chk; float hum; //Stores humidity value<br>
float temp; //Stores temperature value<br>
void setup()<br>
{<br>
Serial.begin(9600);<br>
dht.begin();<br>
}<br>
void loop()<br>
{<br>
delay(2000);<br>
//Read data and store it to variables hum and temp<br>
hum = dht.readHumidity();<br>
temp= dht.readTemperature();<br>
//Print temp and humidity values to serial monitor<br>
Serial.print("Humidity: ");<br>
Serial.print(hum);<br>
Serial.print(" %, Temp: ");<br>
Serial.print(temp);<br>
Serial.println(" Celsius");<br>
delay(1000); //Delay 2 sec.<br>
}<br>
 
 
 😎IR😎<br>
 int ir=D7;<br>
int led=D5;<br>
void setup() {<br>
// put your setup code here, to run once:<br>
pinMode(ir,INPUT);<br>
pinMode(led,OUTPUT);<br>
Serial.begin(9600);<br>

}<br>

void loop() {<br>
// put your main code here, to run repeatedly:<br>
int irvalue=digitalRead(ir);<br>
if(irvalue==LOW)<br>
{<br>
Serial.println("LOW");<br>
digitalWrite(led,HIGH);<br>
}<br>
else<br>
{<br>
Serial.println("HIGH");<br>
digitalWrite(led,LOW);<br>
}<br>
delay(100);<br>
}<br>
 

 
                  
                     
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////<br>                   
                     
//Chassing LED//
int pinsCount=7; // declaring the integer variable pinsCount
int pins[] = {D0,D1,D2,D3,D4,D5,D6}; // declaring the array pins[]

void setup() {
for (int i=0; i<pinsCount; i=i+1){ // counting the variable i from 0 to 9
pinMode(pins[i], OUTPUT); // initialising the pin at index i of the array of pins as OUTPUT
}
}
void loop() {
for (int i=0; i<pinsCount; i=i+1){ // chasing right
digitalWrite(pins[i], HIGH); // switching the LED at index i on
delay(100); // stopping the program for 100 milliseconds
digitalWrite(pins[i], LOW); // switching the LED at index i off
}
for (int i=pinsCount-1; i>0; i=i-1){ // chasing left (except the outer leds)
digitalWrite(pins[i], HIGH); // switching the LED at index i on
delay(100); // stopping the program for 100 milliseconds
digitalWrite(pins[i], LOW); // switching the LED at index i off

}
}
                 
