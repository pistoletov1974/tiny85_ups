/* ATtiny85 as an I2C Master  Ex1          BroHogan                      1/21/11
 * I2C master reading DS1621 temperature sensor. (display with leds)
 * SETUP:
 * ATtiny Pin 1 = (RESET) N/U                      ATtiny Pin 2 = (D3) LED3
 * ATtiny Pin 3 = (D4) to LED1                     ATtiny Pin 4 = GND
 * ATtiny Pin 5 = SDA on DS1621                    ATtiny Pin 6 = (D1) to LED2
 * ATtiny Pin 7 = SCK on DS1621                    ATtiny Pin 8 = VCC (2.7-5.5V)
 * NOTE! - It's very important to use pullups on the SDA & SCL lines!
 * DS1621 wired per data sheet. This ex assumes A0-A2 are set LOW for an addeess of 0x48
 * TinyWireM USAGE & CREDITS: - see TinyWireM.h
 * NOTES:
 * The ATtiny85 + DS1621 draws 1.7mA @5V when leds are not on and not reading temp.
 * Using sleep mode, they draw .2 @5V @ idle - see http://brownsofa.org/blog/archives/261
 */

#include <TinyWireM.h>                  // I2C Master lib for ATTinys which use USI
#include <SoftwareSerial.h>
//#include <Adafruit_INA219.h>
//#include <INA.h>       
//use SNAPDUINO BOARD 
SoftwareSerial Serial(3, 4); // RX and TX

#define INA219        0x40              // 7 bit I2C address for DS1621 temperature sensor
#define INPUT_PIN         3              // ATtiny Pin  1 
#define OUTPUT_PIN         1              // ATtiny Pin  6



int16_t current,current_prev , current_filtered;
uint16_t voltage,voltage_prev;

boolean power=false;
boolean power_prev=power;
boolean output=true;
String state;
uint8_t delay_cycles_on=0, delay_cycles_off=0;
uint8_t  print_cycles=0; //to reduce printing to uart
uint8_t  print_off_cycles=1;


void setup(){
  pinMode(OUTPUT_PIN,OUTPUT);
  Serial.begin(9600);
  
  pinMode(INPUT_PIN,INPUT);
  delay(100);
  
  output=true;


  TinyWireM.begin();                    // initialize I2C lib
  
  TinyWireM.beginTransmission(INA219);
  TinyWireM.send(0x00);                 // Access Command Register
  TinyWireM.send(0x1F);                //  1FFF to control register
  TinyWireM.send(0xFF);                // 
  TinyWireM.endTransmission();          // Send to the slave
  delay(1);

  TinyWireM.beginTransmission(INA219);
  TinyWireM.send(0x05);                 // Access  Register
  TinyWireM.send(0x10);                //  4096 dec to calibration register
  TinyWireM.send(0x00);                // 
  TinyWireM.endTransmission();
  delay(1);

  //print calibration register
  TinyWireM.beginTransmission(INA219);
  TinyWireM.send(0x05);                 // read current
  TinyWireM.endTransmission();          // Send 1 byte to the slave
  delay(1);

   TinyWireM.requestFrom(INA219,1);
   Serial.println(TinyWireM.receive());
   delay(5000);
   digitalWrite(OUTPUT_PIN,HIGH); //power on device
   delay(15000);
   

 
}


void loop(){




  power=!digitalRead(INPUT_PIN);  

  
  // get cuurent
  TinyWireM.beginTransmission(INA219);
  TinyWireM.send(0x04);                 // read current 0x04  current register
  TinyWireM.endTransmission();          // Send 1 byte to the slave
  delay(1);
 
  TinyWireM.requestFrom(INA219,2); // Request 2 byte from slave
  current = TinyWireM.receive();
  current= current<<8;
  current |= TinyWireM.receive(); 
 

  // get voltage
  TinyWireM.beginTransmission(INA219);
  TinyWireM.send(0x02);                 // read current
  TinyWireM.endTransmission();          // Send 1 byte to the slave
  delay(1);
 
  TinyWireM.requestFrom(INA219,2); // Request 1 byte from slave
  voltage = TinyWireM.receive();
  voltage= voltage<<8;
  voltage |= TinyWireM.receive(); 
  voltage = (voltage>>3)*4;
 
   current_filtered = (1-0.05)*current_filtered + 0.05*current;
  

  // process logic 
//add current check 
// if current < 4800  and current >1000

  
  if (  (((current_filtered - current)<2000) && output && !power)  )   {
 
      output=false;
      delay_cycles_off=0;
      digitalWrite(OUTPUT_PIN,LOW); //shutdown device 
    
  } 



  
  

  if (!power_prev && power) delay_cycles_on=0; //power 220v going to ON
  if ((!output) && power) delay_cycles_on++; 
   if (abs(current-current_prev)>100) print_cycles=0;
   if (abs(voltage-voltage_prev)>100) print_cycles=0;
   
  if (delay_cycles_on>10) {
    output=true;
    print_cycles=250;
    print_off_cycles=250;
    delay_cycles_on=0;
    digitalWrite(OUTPUT_PIN,HIGH); //power ON device
  }

 


  power_prev=power;
  voltage_prev=voltage;
  current_prev=current;
  //decrease print frequent
 if ((print_cycles==0) && (print_off_cycles==0)) {
  // output states
  print_cycles = 250;
  Serial.print(current);
  Serial.print(";"); 
  Serial.print((int16_t)voltage);
  Serial.print(";");
  state=(power)?"ON":"OFF";
  Serial.print(state);
  Serial.print(";");
  state=(output)?"ON":"OFF";
  Serial.print(state);
  Serial.print(";");
  Serial.println(current_filtered);
 }
  
 print_cycles--; 
 //print_off_cycles using for off printing to uart 
 print_off_cycles=(print_off_cycles>0)?print_off_cycles-1:0;
  delay(1000);
  
}
