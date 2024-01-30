#include <WiFi.h>

const char *ssid = "069_131_218_548";      // Replace with your desired WiFi network name
const char *password = "12345678";
const int speeed = 75;

volatile int IN1;  // for backward of left side
volatile int IN2;  //  for forward of left side
volatile int IN3;  // for forward of right side
volatile int IN4;  //  for backward of right side

volatile int E12 ;
volatile int E34;

volatile int back_rigt_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int back_rigt_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

volatile int back_left_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int back_left_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

volatile int back_center_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int back_center_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

volatile int wenger_rigt_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int wenger_rigt_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

volatile int wenger_left_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int wenger_left_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

volatile int center_ferword_trig ; // ESP32 pin GPIO23 connected to Ultrasonic Sensor's TRIG pin
volatile int center_ferword_echo ; // ESP32 pin GPIO22 connected to Ultrasonic Sensor's ECHO pin

bool start = true ; 
bool parallel = true ;
int hight = 30 ;
int width = 20 ; 

void left_side_forward(int speed) {
  // Left
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  analogWrite(E12, speed);
}

void right_side_forward(int speed) {
  // Right
  digitalWrite(IN3, 1);
  digitalWrite(IN4`, 0);
  analogWrite(E34, speed * 0.86);
}

void left_side_backward(int speed) {
  // Left
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  analogWrite(E12, speed);
}

void right_side_backward(int speed) {
  // Right
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  analogWrite(E34, speed);
}

void forward(int speed) {
  left_side_forward(speed);
  right_side_forward(speed);
}

void backward(int speed) {
  left_side_backward(speed);
  right_side_backward(speed);
}

void stop_car() {
  left_side_forward(0);
  right_side_forward(0);
}

void left_forward(int speed, int angle) {
  if (angle < 45) {
    left_side_forward(speed * ((45-angle)/45) );
    right_side_forward(speed);
  } else {
    left_side_backward(speed * (angle/90) );
    right_side_forward(speed);
  }
}

void left_backward(int speed, int angle) {
  if (angle < 45) {
    left_side_backward(speed * ((45-angle)/45) );
    right_side_backward(speed);
  } else {
    left_side_forward(speed * (angle/90) );
    right_side_backward(speed);
  }
}

void right_forward(int speed, int angle) {
  if (angle < 45) {
    left_side_forward(speed);
    right_side_forward(speed * ((45-angle)/45) );
  } else {
    left_side_forward(speed);
    right_side_backward(speed * (angle/90) );
  }
}

void right_backward(int speed, int angle) {
  if (angle < 45) {
    left_side_backward(speed);
    right_side_backward(speed * ((45-angle)/45) );
  } else {
    left_side_backward(speed);
    right_side_forward(speed * (angle/90) );
  }
}

void park_right(){
  forward(speeed);
  delay(300);
  left_forward(speeed,45);
  delay(500);
  right_backward(speeed,22);
  delay(500);
  while(read_dist( back_center_trig,back_center_echo) > 5) {
    left_backward(speeed,45);
  }
  while(rea1d_dist( back_center_trig,back_center_echo)-3 <= read_dist( center_ferword_trig,center_ferword_echo) && read_dist( center_ferword_trig,center_ferword_echo) <= read_dist( back_center_trig,back_center_echo)+3) {
    forward(speeed);
  }
  stop_car(); 
  start = false;
}

void park_left(){
  forward(speeed);
  delay(300);
  right_forward(speeed,45);
  delay(500);
  left_backward(speeed,22);
  delay(500);
  while(read_dist( back_center_trig,back_center_echo) > 5) {
    right_backward(speeed,45);
  }
  while(read_dist( back_center_trig,back_center_echo)-3 <= read_dist( center_ferword_trig,center_ferword_echo) <= read_dist( back_center_trig,back_center_echo)+3) {
    forward(speeed);
  }
  stop_car(); 
  start = false;
}

void park_right2 (){
  backward(speeed);
  delay(300);
  right_forward(speeed,90);
  delay(500);
  while(read_dist( center_ferword_trig,center_ferword_echo) > 5) {
      forward(speeed);
  }
  stop_car();
  start = false;
}

void park_left2(){
  backward(speeed);
  delay(300);
  left_forward(speeed,90);
  delay(500);
  while(read_dist( center_ferword_trig,center_ferword_echo) > 5) {
      forward(speeed);
  }
  stop_car();
  start = false;
}

float read_dist(int TRIG_PIN,int ECHO_PIN){
  float duration_us, distance_cm;
    // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
   return distance_cm;

}

float calculateDistance(int rssi) {
  // Use a simple formula to estimate distance based on RSSI
  // This formula is just an example and may not be accurate in all scenarios
  // You may need to calibrate it based on your specific environment
  // The exponent (2.4) and the constants (-45 and 20) are just placeholders
  return pow(10, ((-45 - rssi) / (10 * 2.4)));
}

void wifi_scan (){
  int numNetworks = WiFi.scanNetworks();
  Serial.println("Scan completed!");

  if (numNetworks == 0) {
    Serial.println("No networks found");
  } else {
    for (int i = 0; i < numNetworks; i++) {
      // Print SSID and RSSI for each network found
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID(i));
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI(i));

      // Use RSSI for proximity estimation (distance calculation)
      int rssi = WiFi.RSSI(i);
      float distance = calculateDistance(rssi);

      Serial.print("Estimated Distance: ");
      Serial.println(distance);

      Serial.println("-----");
    }
  }
}

void setup() {
  IN1 = 26;
  IN2 = 27;
  IN3 = 14;
  IN4 = 12;

  E12 = 25;
  E34 = 13;

  back_rigt_trig =23 ; 
  back_rigt_echo = 22 ; 

  back_left_trig =18; 
  back_left_echo =5; 

  back_center_trig = 21; 
  back_center_echo = 19; 

  wenger_rigt_trig =15; 
  wenger_rigt_echo =35; 

  wenger_left_trig =4;
  wenger_left_echo =2; 

  center_ferword_trig =32; 
  center_ferword_echo =33; 

  pinMode(back_rigt_trig, OUTPUT);
  pinMode(back_left_trig, OUTPUT);
  pinMode(back_center_trig, OUTPUT);
  pinMode(wenger_rigt_trig, OUTPUT);
  pinMode(wenger_left_trig, OUTPUT);
  pinMode(center_ferword_trig, OUTPUT);

  pinMode(back_rigt_echo, INPUT);
  pinMode(back_left_echo, INPUT);
  pinMode(back_center_echo, INPUT);
  pinMode(wenger_rigt_echo, INPUT);
  pinMode(wenger_left_echo, INPUT);
  pinMode(center_ferword_echo, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);  // open the serial port at 9600 bps:

  // Set up the ESP32 as an access point
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void loop() {

if (start){
  if (parallel ) {
    float wenger_rigt = read_dist( wenger_rigt_trig,wenger_rigt_echo);
    float back_rigt = read_dist( back_rigt_trig,back_rigt_echo);
    
    float wenger_left = read_dist( wenger_left_trig,wenger_left_echo);
    float back_left = read_dist( back_left_trig,back_left_echo);
    
    float right = (wenger_rigt + back_rigt ) / 2 ; 
    float left = (wenger_left + back_left ) /2 ;

    if(right < left) {
      while(!(wenger_rigt >= width && back_rigt >= width)) {
          forward(speeed);
      }
      park_right();
    } else {
      while(!(wenger_left >= width && back_left >= width)) {
          forward(speeed);
      }
      park_left();
    }
  } else {
      float wenger_rigt = read_dist( wenger_rigt_trig,wenger_rigt_echo);
      float back_rigt = read_dist( back_rigt_trig,back_rigt_echo);
    
      float wenger_left = read_dist( wenger_left_trig,wenger_left_echo);
      float back_left = read_dist( back_left_trig,back_left_echo);
    
      float right = (wenger_rigt + back_rigt ) / 2 ; 
      float left = (wenger_left + back_left ) /2 ;

      if(right < left) {
        while(!(wenger_rigt >= hight || back_rigt >= hight)) {
            forward(speeed);
        }
        if(wenger_rigt >= hight) {
          forward(speeed);
          delay(300);
          if(wenger_rigt >= hight) {
            park_right2();
          }
        }
        if(back_rigt >= hight) {
          backward(speeed);
          delay(300);
          if(back_rigt >= hight) {
            backward(speeed);
            delay(600);
            park_right2();
          }
        }
      } else {
        while(!(wenger_left >= hight || back_left >= hight)) {
            forward(speeed);
        }
        if(wenger_left >= hight) {
          forward(speeed);
          delay(300);
          if(wenger_left >= hight) {
            park_left2();
          }
        }
        if(back_left >= hight) {
          backward(speeed);
          delay(300);
          if(back_left >= hight) {
            backward(speeed);
            delay(600);
            park_left2();
          }
        }
      }
    }
  }
}
