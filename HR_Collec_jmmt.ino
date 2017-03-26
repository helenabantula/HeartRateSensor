/**
 *
 * Project: Heart Rate Measurement for Caldea application
 *
 */

#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet2.h>


EthernetClient client;
byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xCB, 0x2D};
IPAddress ip(192, 168, 1, 177);
IPAddress server(192, 168, 1, 109);
unsigned int  serverPort = 11999;


#define MAX_DEVIATION 10
#define ABS_HR_MAX 180
#define ABS_HR_MIN 30
#define BUFFER_LENGTH 100 //buffer length of 100 stores 4 seconds of samples running at 25sps
#define HUMAN_YES 90000
#define HUMAN_NO 30000

bool persona = false; 
uint32_t aun_ir_buffer[BUFFER_LENGTH]; //infrared LED sensor data
uint32_t red_Led = 0;  //red LED sensor data

int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid

int8_t  maxHR = 120;  
int8_t  minHR = 50;
int8_t  defaultHR = 75;

volatile unsigned int timeCount=0;


/**
 * Interrupt from Timer 1 Compare Register A
 * It just increments timeCount to check conditions in main code
**/
ISR(TIMER1_COMPA_vect){
  timeCount++;
}


/**
 * Interrupt from Timer 1 Compare Register B
 * It checks for new data from Server
**/
ISR(TIMER1_COMPA_vect){
  if(client.available()){
    char readedData = client.read();
    //Serial.println(readedData);
    if(readedData =='A'){
      digitalWrite(8, HIGH);  
      delay(100);     //Assegurar que està bé...
    }   
    else
    digitalWrite(8, LOW);
  }
}


/**
 * Setup function: this routine runs once when the sytem starts
 *    -Sets up timer 1 to provide:
 *        -On Channel A an interrupt at 5Hz. Used to check client connection and send 'Alive'
 *        -On Channel B an interrupt at 5Hz. Used to check for new data from Master
 *    -Configure I/O pins. D9 for HR sensor interrupt. D8 for vibration motor control
 *    -Init serial communications (They are needed to connect Atmega328 to Ethernet shield using SPI)
 *    -Init Ethernet
 *    -Init HR sensor
 *
 *  Note clock speed is assumed to be 16MHz
**/
void setup() {
  uint8_t uch_dummy;

  //Set up Timer 1
  TCCR1A = 0;          // normal operation
  TCCR1B = bit(WGM12) | bit(CS10) | bit (CS12);   // CTC, scale to clock / 1024
  OCR1A = 3125;       //compare A register value (clock speed / 1024 * 3125) -> Aprox 5Hz
  OCR1B = 3125;
  TIMSK1 = 6; //interrupt on Compare A and B Match (00000110)

  //Configure I/O pins
  pinMode(9, INPUT);  //pin D9 connects to the interrupt output pin of the MAX30102
  pinMode(8, INPUT);  //pin D8 detects when there is somebody

  //Init needed functionalities in Atmega328
  Serial.begin(115200); //Initialize serial communication at 115200 bits per second
  delay(500);

  //Ethernet
  Ethernet.begin(mac, ip); //This inits as well the SPI
  delay(500);

  //Initialize the MAX30102
  maxim_max30102_reset(); 
  delay(250);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);
  delay(100);
  maxim_max30102_init();
}


/**
 * Loop function: Called infinitely
 *    -If Ethernet client is NOT connected, we check the time elapsed from last check,
 *     and if it is >10seconds we try to connect again
 *    -If Ethernet client is connected:
 *      -If there is no person touching the sensor
 *          -Check if we need to end an "Alive" signal. Done every 30 seconds 
 *          -Read 4 data (Red LED) from sensor and check if there is a person. This takes 160ms
 *          -If there is, reset maxHR and minHR; set persona=True; Signal to the master
 *          -If there is not, make the red LED blink: 1.5 seconds OFF, 0.5 seconds ON
 *      -If there is a person touching the sensor
 *          -Take 4 seconds of data from sensor and make first estimate of HR
 *          -Check limits of estimated HR and update maxHR and minHR
 *          -Send HR + ID to Master
 *          -While there is a person touching the sensor do:
 *              -Check if the person follows there. If not, persona=FALSE and notifiy the Master
**/
void loop(){
  uint32_t dummy;
  uint8_t i;
  String ID = "R";    //no pot ser un char, s'ha de poder sumar quan s'envia juntament amb HR
  
  if (!client.connected()){
    if(timeCount>=50){  //This will be 10 seconds
      noInterrupts();
      timeCount=0;
      interrupts();        
      //Serial.println("Re-connecting");
      client.flush();
      client.stop();  //This waits by default 1 second
      client.connect(server, serverPort);
    }
  }
  else{
    if(!persona){
      if(timeCount>=150){  //This will be 30s
        noInterrupts();
        timeCount=0;
        interrupts(); 
        //Serial.println("Alive");
        client.println("Alive");
      }
      //Serial.println(persona);
      //Serial.println(red_Led);
      for(i=0; i<4;i++){
        while(digitalRead(9)==1);  //wait until the interrupt pin detects new data from sensor
        maxim_max30102_read_fifo((&red_Led), (&dummy));  //read from MAX30102 FIFO  
        //Serial.println(red_Led);
      }
      if(red_Led >= HUMAN_YES){
        maxHR = 120;  
        minHR = 50;
        persona = true;
        client.println("Y0");
        //client.println("R300"); //no estressar el motor mentre calcula els 4s...
        //fer algo amb el motor ----> usar delays?
      }
      else{
        maxim_max30102_write_reg(REG_MODE_CONFIG,131);
        delay(1500);
        maxim_max30102_write_reg(REG_MODE_CONFIG,3);
        delay(500);
      }
    }
    else{
      //Read BUFFER_LENGTH*Sample Time seconds of data previous to make first HR estimation
      for(i=0;i<BUFFER_LENGTH;i++){
        while(digitalRead(9)==1);  //wait until the interrupt pin asserts
        maxim_max30102_read_fifo((&red_Led), (aun_ir_buffer+i));  //read from MAX30102 FIFO
        //Serial.print(F("red="));
        //Serial.print(red_Led, DEC);
        //Serial.print(F(", ir="));
        //Serial.println(aun_ir_buffer[i], DEC);
      }
   
      //calculate heart rate after first 100 samples (first 4 seconds of samples)
      maxim_heart_rate(aun_ir_buffer, BUFFER_LENGTH, &n_heart_rate, &ch_hr_valid); 

      if((n_heart_rate < minHR) || (n_heart_rate > maxHR))
        n_heart_rate=defaultHR;
      else{
        maxHR = n_heart_rate + MAX_DEVIATION;
        if(maxHR>ABS_HR_MAX) maxHR=ABS_HR_MAX;
        minHR = n_heart_rate - MAX_DEVIATION;
        if(minHR<ABS_HR_MIN) minHR=ABS_HR_MIN;
      }
   
      client.println(ID+n_heart_rate);

      //Continuously taking samples from MAX30102.  Heart rate is calculated every 1 second.
      while(persona && client.connected()){
        if(red_Led < HUMAN_NO){
          persona = false;
          client.println("Z0");
          break;
        }

        //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
        for(i=35;i<BUFFER_LENGTH;i++)
          aun_ir_buffer[i-35]=aun_ir_buffer[i];

        //take 25 sets of samples before calculating the heart rate.
        for(i=64;i<BUFFER_LENGTH;i++){
          while(digitalRead(9)==1);
          maxim_max30102_read_fifo((&red_Led), (aun_ir_buffer+i));
          //send samples and calculation result to terminal program through UART
          //Serial.print(F("red="));
          //Serial.print(red_Led, DEC);
          //Serial.print(F(", ir="));
          //Serial.print(aun_ir_buffer[i], DEC);
          //Serial.print(F(", HR="));
          //Serial.print(n_heart_rate, DEC);
          //Serial.print(F(", HRvalid="));
          //Serial.println(ch_hr_valid, DEC);
        }
        maxim_heart_rate(aun_ir_buffer, BUFFER_LENGTH, &n_heart_rate, &ch_hr_valid); 
     
        // POST FILTERING
        if((n_heart_rate < minHR) || (n_heart_rate > maxHR))
          n_heart_rate = defaultHR;
        else{
          maxHR = n_heart_rate + MAX_DEVIATION;
          if(maxHR>ABS_HR_MAX) maxHR=ABS_HR_MAX;
          minHR = n_heart_rate - MAX_DEVIATION;
          if(minHR<ABS_HR_MIN) minHR=ABS_HR_MIN;
        }
        client.println(ID+n_heart_rate);
      }
    }
  }
}
