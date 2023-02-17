/*This is team Hoang, Kien, Duc, Thu's project

  The Device follows these steps:
  1. Instruct visitor to stand at the optimal distance for mask_scanning (35 - 60 cm)
  2. Check if visitor is wearing a mask via ESP-32 Camera (Mask or Not)
  3. Automatically dispense hand soap
*/


#include <Wire.h> //allows you to communicate with I2C/TWI devices
#include <LiquidCrystal_I2C.h> // library to use LCD display
#include <Servo.h> // library to control servo
#include <Ultrasonic.h> // library to control ultrasonic distance sensor
#include <Adafruit_MLX90614.h> // library to control temperature sensor
 
#define SERVO_PIN_1       9 // open/close door
#define SERVO_PIN_2       10 // 
#define GREEN_LED_PIN   7 
#define RED_LED_PIN     6

#define TRIG_PIN_1       12   // for ultrasonic 1 
#define ECHO_PIN_1       13   // for ultrasonic 1
#define TRIG_PIN_2       4    // for ultrasonic 2
#define ECHO_PIN_2       5    // for ultrasonic 2


#define START_RUN        'a' // instruct camera to start scanning
#define RESTART_CAM      'r' // restart camera -- start server 
#define RESTART_SCAN     'n'  // restart scaning

#define CLOSE_DOOR_VAL        180
#define OPEN_DOOR_VAL         90
#define DOOR_LOCK_VAL         0
   
#define MASK_THRESHOLD        80 // percent confidence that mask is on

// temperature allowed
// #define MAX_TEMP    39   
// #define MIN_TEMP    35
// temperature sensor states
// #define NO_OBJECT     -1
// #define TEMP_LOW       0
// #define TEMP_HIGH      1
// #define TEMP_ALLOWED   2


// optimal distance for face scanning
#define MIN_DISTANCE    35 
#define MAX_DISTANCE    60

// states
#define NO_OBJECT_RESULT  0
#define PASS_RESULT       1
#define ERROR_RESULT      2

LiquidCrystal_I2C my_lcd(0x27,16,2);   // set the LCD address to 0x27 for a 16 chars and 2 line display
Ultrasonic my_ultrasonic_1(TRIG_PIN_1, ECHO_PIN_1);
Ultrasonic my_ultrasonic_2(TRIG_PIN_2, ECHO_PIN_2);
// Adafruit_MLX90614 my_temp_sensor = Adafruit_MLX90614(); // temp sensor
Servo my_servo_1;
Servo my_servo_2;

char IP_add[20]; // define a string with 20 characters 

unsigned long gulStart_Timer_LCD = 0; // 0 to 4,294,967,295
unsigned long gulRestart_Timer = 0;
unsigned long gulDistance_Timer = 0;

bool is_camera_on = false;
bool is_ultrasonic_on = false;
bool is_servo_1_on = false;
bool is_servo_2_on = false;

int check_count = 0; // conditions met
unsigned short IsNeed_Restart = 0;
unsigned long Restart_Timer = 0;
unsigned long Start_Timer_LCD = 0;

void setup() {
  char data[30];
  unsigned short Exit = 0;
  unsigned short data_len = 0; 
  short a = 0; // -32,767 to 32,767 

// setup sensors and devices
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  LED_controller(NO_OBJECT_RESULT);
  servo_1_controller(DOOR_LOCK_VAL); 
  my_lcd.init();
  my_lcd.backlight();
  my_lcd.clear();
  Serial.begin(9600);
  
  LED_controller(NO_OBJECT_RESULT);
  servo_1_controller(DOOR_LOCK_VAL);

  memset(data, '\0', sizeof(data));
  memset(IP_add, '\0', sizeof(IP_add));

  my_lcd.clear();
  my_lcd.print("Connecting...");
  do
  { 
    data_len = read_serial_data(data, sizeof(data));
    
    if(data_len > 0)
    {
      for(short i=0; i<data_len; i++)
      {
        if(data[i] == '#')
        {
          i++;
          while(data[i] != ',')
          {
            IP_add[a] = data[i++];
            a++;
          }
          Exit = 1;
          break;
        }
      }
    }
    else
    {
      if(IsNeed_Restart == 0)
      {
        Serial.println(RESTART_CAM);

        IsNeed_Restart = 1;
      }
      
      if((millis() - Restart_Timer) > 5000)
      {
        IsNeed_Restart = 0;
        Restart_Timer = millis();
      }
    }
  }while(Exit == 0);

  LCD_display_IP(IP_add);
  my_lcd.clear();
  my_lcd.print("ok");
  is_ultrasonic_on = true;
}

void loop(){
  if(is_ultrasonic_on && check_count == 0){
    my_lcd.clear();
    my_lcd.print("Ultrasonic");
    my_lcd.setCursor(0,1);
    my_lcd.print("scanning");

    delay(500);

    LED_controller(ERROR_RESULT);
    unsigned short current_distance = 0;

    current_distance = get_distance_1();

    if(current_distance > 0){
        if(current_distance < MAX_DISTANCE && current_distance > MIN_DISTANCE){
        my_lcd.clear();
        my_lcd.print("Scanning..");
        LED_controller(PASS_RESULT);
        check_count += 1;
        is_camera_on = true;
        
        delay(1000);
      } else if(current_distance > MAX_DISTANCE){
        my_lcd.clear();
        my_lcd.print("Come closer, ");
        my_lcd.setCursor(0,1);
        my_lcd.print("U R ");
        my_lcd.print(current_distance);
        my_lcd.print("cm away ");
      
        LED_controller(ERROR_RESULT);
  
        delay(2000);
      } else if(current_distance < MIN_DISTANCE){
        my_lcd.clear();
        my_lcd.print("Back up,");
        my_lcd.setCursor(0,1);
        my_lcd.print("U R ");
        my_lcd.print(current_distance);
        my_lcd.print("cm away");
        
        delay(2000);
      }
    } else {
      LED_controller(ERROR_RESULT);
      my_lcd.print("No Object");    
    }
  }

  if(is_camera_on && check_count == 1){
    my_lcd.clear();
    my_lcd.print("Camera running");
    char data[30];
    unsigned short data_len = 0;
    short mask_percent = 0;

    my_lcd.clear();
    my_lcd.print("Scanning mask");
    LED_controller(ERROR_RESULT);
    delay(2000);
    //LED_controller(ERROR_RESULT);
    Serial.println(START_RUN); //start scanning

    memset(data, '\0', sizeof(data));
    data_len = read_serial_data(data, sizeof(data));
 
    if(data_len > 0){
      if(data[0] == '*'){
        sscanf(data, "*%d,", &mask_percent); // gets mask percent from esp32
      }
      my_lcd.clear();
      my_lcd.print("Mask percent:");
      my_lcd.setCursor(0,1);
      my_lcd.print(mask_percent);
      my_lcd.print("%");

      delay(3000);
    }
  
    if(mask_percent > MASK_THRESHOLD){
      LED_controller(PASS_RESULT);
      my_lcd.setCursor(0,1);
      my_lcd.print("Mask_detected");
 
      check_count += 1;
      is_servo_2_on = true; 
      
      delay(1000);
    } else {
      LED_controller(ERROR_RESULT);
      my_lcd.clear();
      my_lcd.print("No mask detected");
      
      delay(1000);
    }
  }

  if(is_servo_2_on && check_count == 2){
    unsigned short hand_distance = get_distance_2();
    my_lcd.clear();
    my_lcd.print("Soap dispenser");
    my_lcd.setCursor(0,1);
    my_lcd.print("running");
    
    if (hand_distance < 10){
      LED_controller(PASS_RESULT);
      my_lcd.clear();
      my_lcd.println("Dispensing soap... ");

      check_count += 1;
      is_servo_1_on = true;
      servo_2_controller(OPEN_DOOR_VAL);
      delay(2000);
      servo_2_controller(CLOSE_DOOR_VAL);  
    } else {
      LED_controller(ERROR_RESULT);
    }
    delay(500);
  }

  if (is_servo_1_on && check_count == 3){
    LED_controller(PASS_RESULT);
    check_count = 0;
    my_lcd.clear();
    my_lcd.print("All good");
    my_lcd.setCursor(0,1);
    my_lcd.print("have a nice day");
   
    delay(2000);
    servo_1_controller(OPEN_DOOR_VAL);
    delay(4000);
    servo_1_controller(CLOSE_DOOR_VAL);
  }
} 


// defining functions:

void LED_controller (short scan_status) {
  /* if visitor is good to go, GREEN 
   * if visitor fails to meet all requirements, RED
   * else, nothing turns on
   */
  if(scan_status == PASS_RESULT) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  } 
  else if(scan_status == ERROR_RESULT) {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  }
  else {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void servo_1_controller(int door_val) {
  // moves the servo as instructed
  
  my_servo_1.attach(SERVO_PIN_1);

  if(door_val == OPEN_DOOR_VAL) {
    for(int i = CLOSE_DOOR_VAL; i > OPEN_DOOR_VAL; i--) {
      my_servo_1.write(i);
      delay(20);
    }
  }
  else if(door_val == CLOSE_DOOR_VAL) {
    for(int i = OPEN_DOOR_VAL; i < CLOSE_DOOR_VAL; i++){
      my_servo_1.write(i);
      delay(20); 
    }
  }
  else if(door_val == DOOR_LOCK_VAL){
    my_servo_1.write(CLOSE_DOOR_VAL);
  }

  my_servo_1.detach();
}

void servo_2_controller(int door_val) {
  // moves the servo as instructed
  
  my_servo_2.attach(SERVO_PIN_2);

  if(door_val == OPEN_DOOR_VAL) {
    for(int i = CLOSE_DOOR_VAL; i > OPEN_DOOR_VAL; i--) {
      my_servo_2.write(i);
      delay(20);
    }
  }
  else if(door_val == CLOSE_DOOR_VAL) {
    for(int i = OPEN_DOOR_VAL; i < CLOSE_DOOR_VAL; i++){
      my_servo_2.write(i);
      delay(20); 
    }
  }
  else if(door_val == DOOR_LOCK_VAL){
    my_servo_2.write(CLOSE_DOOR_VAL);
  }

  my_servo_2.detach();
}

unsigned short read_serial_data(char *data, short data_size) {
  /*
   * This function reads serial data from camera
   */
    
  short i=0;

  if(Serial.available()) {
    *(data+i) = Serial.read();
    i++;
    delay(2);
   
    while(Serial.available()){
      *(data+i) = Serial.read();
      i++;

      if(i >= data_size){
        break;
      }
      delay(2);
   }
  }

  Serial.flush();

  return i;
}

void LCD_display_IP(char *ip) {
    my_lcd.clear();
    my_lcd.setCursor(0,0);
    my_lcd.print("CONNECTED TO:");
    my_lcd.setCursor(0,1);
    my_lcd.print(ip);
    delay(3000);
}

unsigned short get_distance_1() {
  /*
   * Reads the distance of the object.
   */
  unsigned short face_distance = 0; 

  face_distance = my_ultrasonic_1.read();
  delay(50);

  return face_distance;
}

unsigned short get_distance_2() {
  /*
   * Reads the distance of the object.
   */
  unsigned short face_distance = 0; 

  face_distance = my_ultrasonic_2.read();
  delay(50);

  return face_distance;
}

void LCD_disp_mask(short sMask_Percent) {
  my_lcd.clear();
  my_lcd.print("Mask: ");
  my_lcd.print(sMask_Percent);
  my_lcd.print("%");
  my_lcd.setCursor(0,1);

  if(sMask_Percent >= MASK_THRESHOLD)
  {
    my_lcd.print("Enter Allowed.");
  }
  else
  {
    my_lcd.print("PLEASE WEAR MASK");
  }
}


// component testing code

/* void loop(){
 *  
  //this is to test servo
  
  servo_1_controller(OPEN_DOOR_VAL);
  delay(2000);
}*/

/*void loop(){
  //this is to test camera
  char data[30];
  unsigned short data_len = 0;
  short mask_percent = 0;
  
  Serial.println(START_RUN);
  
  data_len = read_serial_data(data, sizeof(data));

  if(data_len > 0){
    if(data[0] == '*'){
      sscanf(data, "*%d,", &mask_percent); // gets mask percent from esp32
    }
  }

  if(mask_percent > MASK_THRESHOLD){
    LED_controller(PASS_RESULT);
    delay(1000);
  } else {
    LED_controller(ERROR_RESULT);
    delay(1000);
  }
}*/

/*void loop(){
   // this to test led
   servo_1_controller(OPEN_DOOR_VAL);
   delay(2000);
   LED_controller(PASS_RESULT);
   delay(2000);  
   LED_controller(ERROR_RESULT);
   delay(2000);
}*/

/*void loop(){
  // this is to test temp sensor --- BROKEN
  LED_controller(PASS_RESULT);
  delay(500);
  LED_controller(ERROR_RESULT);
  delay(500);
   
  float object_temp = 0;

  object_temp = my_temp_sensor.readObjectTempC();

  if (object_temp > 5.0){
    LED_controller(PASS_RESULT);
    delay(2000);
  } else {
    LED_controller(ERROR_RESULT);
    delay(2000);
  }
}*/


/* void loop(){
  // this is to test lcd  -- SOMETIMES WORKs

  unsigned short test = 0;
  test = get_distance();

  LED_controller(PASS_RESULT);
  delay(200);
  LED_controller(NO_OBJECT_RESULT);
  my_lcd.clear();
  my_lcd.print("Hello");
  my_lcd.print(test);
  Serial.print(test);
  my_lcd.setCursor(0,1);
  my_lcd.print("Hoang");

  delay(3000);
  LED_controller(ERROR_RESULT);
  delay(200);
}*/

/* void loop(){
  // this is to test ultrasonic
  unsigned short test = 0;
  LED_controller(PASS_RESULT);
  delay(500);
  LED_controller(ERROR_RESULT);
  delay(500);

  test = get_distance();
  Serial.print("ok1");
  Serial.print(test);
  Serial.print("ok2");

  if (test > 60){
    LED_controller(ERROR_RESULT);
    delay(2000);
  } else {
    LED_controller(PASS_RESULT);
    delay(2000);
  }
  delay(1000);
}*/




/* 
  if (is_temp_sensor_on && check_count == 1){
    object_temp = my_temp_sensor.readObjectTempC();

    if(object_temp > MAX_TEMP) {
      LED_controller(ERROR_RESULT);
      
      my_lcd.print("Too hot");
    } else if (object_temp > MIN_TEMP){
      LED_controller(PASS_RESULT);

      check_count += 1;
      my_lcd.print("2/3");
      is_servo_on = true;
    } else {
      my_lcd.print("try again");
    }
  } */
