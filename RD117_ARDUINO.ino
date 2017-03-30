      /** \file RD117_LILYPAD.ino ******************************************************
 *
 * Project: MAXREFDES117#
 * Filename: RD117_LILYPAD.ino
 * Description: This module contains the Main application for the MAXREFDES117 example program.
 *
 * Revision History:
 *\n 1-18-2016 Rev 01.00 GL Initial release.
 *\n
 *
 * --------------------------------------------------------------------
 *
 * This code follows the following naming conventions:
 *
 * char              ch_pmod_value
 * char (array)      s_pmod_s_string[16]
 * float             f_pmod_value
 * int32_t           n_pmod_value
 * int32_t (array)   an_pmod_value[16]
 * int16_t           w_pmod_value
 * int16_t (array)   aw_pmod_value[16]
 * uint16_t          uw_pmod_value
 * uint16_t (array)  auw_pmod_value[16]
 * uint8_t           uch_pmod_value
 * uint8_t (array)   auch_pmod_buffer[16]
 * uint32_t          un_pmod_value
 * int32_t *         pn_pmod_value
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
 IPAddress server(192, 168, 1, 110);
 unsigned int  serverPort = 11999;


 #define MAX_DEVIATION 10
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


 ISR(TIMER1_COMPA_vect){ //Ep, he arribat a 40, incremento una variable
   timeCount++;
 }


// the setup routine runs once when you press reset:
void setup() {
   uint8_t uch_dummy;

     // set up Timer 1
  TCCR1A = 0;          // normal operation
  TCCR1B = bit(WGM12) | bit(CS10) | bit (CS12);   // CTC, scale to clock / 1024
  OCR1A =  3125;       //compare A register value (clock speed / 1024 * 3125) -> Aprox 5Hz
  TIMSK1 = bit (OCIE1A);  //interrupt on Compare A Match

  //resets the MAX30102
  maxim_max30102_reset(); 

  //Init needed functionalities in Atmega328
  Serial.begin(115200); //Initialize serial communication at 115200 bits per second
  pinMode(9, INPUT);  //pin D9 connects to the interrupt output pin of the MAX30102
  pinMode(8, INPUT);  //pin D8 detects when there is somebody
  delay(1000);

  //Reads/clears the interrupt status register in HR sensor
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);


   
   ///////////Canviar loop while per espera de microruptor. Aixo ha de ser una rutina d'interrupcio: quan detecta persona, començo calcul al loop. Rutina interrupcio lligada amb hardware. 
//   while(Serial.available()==0)  //wait until user presses a key
//   {
//     Serial.write(27);       // ESC command
//     Serial.print(F("[2J"));    // clear screen command
//
//     Serial.println(F("Press any key to start conversion"));
//     delay(1000);
//   }
//   uch_dummy=Serial.read();
 ////////////////////////////////////////////////////////////////////////////////////////////////////

 ////////////////////////////// Ethernet /////////////////////////
    Ethernet.begin(mac, ip);
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }

  delay(1000);

  Serial.println("Connecting...");

  if (client.connect(server, serverPort)) {
    Serial.println("Connected");
  } else {
    Serial.println("Connection failed");
  }

 /////////////////////////////////////////////////////////////////
 
  //Initialize the MAX30102
  maxim_max30102_init();
}


void loop() {
  
  uint32_t dummy;
  uint8_t i;
  String ID = "R";    //no pot ser un char, s'ha de poder sumar quan s'envia juntament amb HR
  
  if (!client.connected()){
    if(timeCount>=50){  //This will be 10 seconds
       noInterrupts();
       timeCount=0;
       interrupts();        
       Serial.println("Re-connecting");
       client.flush();
       client.stop();
       delay(50);
       client.connect(server, serverPort);
    }
  }
  else{
      if(timeCount>=150){  //This will be 30s
       noInterrupts();
       timeCount=0;
       interrupts(); 
       Serial.println("Alive");
       client.println("Alive");
    
    }
  
   if (persona){

   //Read BUFFER_LENGTH*Sample Time seconds of data previous to make first HR estimation
   for(i=0;i<BUFFER_LENGTH;i++)
   {
     while(digitalRead(9)==1);  //wait until the interrupt pin asserts
     maxim_max30102_read_fifo((&red_Led), (aun_ir_buffer+i));  //read from MAX30102 FIFO
     
     Serial.print(F("red="));
     Serial.print(red_Led, DEC);
     Serial.print(F(", ir="));
     Serial.println(aun_ir_buffer[i], DEC);
   }
   
   //calculate heart rate after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate(aun_ir_buffer, BUFFER_LENGTH, &n_heart_rate, &ch_hr_valid); 

   if ((n_heart_rate < minHR) || (n_heart_rate > maxHR)){
          n_heart_rate = defaultHR;
   }
   else{
      maxHR = n_heart_rate + MAX_DEVIATION;
      minHR = n_heart_rate - MAX_DEVIATION;
   }
   
   client.println(ID+n_heart_rate);

   //Continuously taking samples from MAX30102.  Heart rate is calculated every 1 second.
   while(persona && client.connected())    //while(persona)
   {
      if (red_Led < HUMAN_NO){
        persona = false;
        client.println("Z0");
        break;
      }

     //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
     for(i=35;i<BUFFER_LENGTH;i++)
     {
       aun_ir_buffer[i-35]=aun_ir_buffer[i];
     }

     //take 25 sets of samples before calculating the heart rate.
     for(i=65;i<BUFFER_LENGTH;i++)
     {
       while(digitalRead(9)==1);

      char readedData = client.read();
      Serial.println(readedData);
      if (readedData =='A'){
          digitalWrite(8, HIGH);  
          delay(100);     //Assegurar que està bé...
      }   
      else
          digitalWrite(8, LOW);
    
       maxim_max30102_read_fifo((&red_Led), (aun_ir_buffer+i));
       
       //send samples and calculation result to terminal program through UART
//       Serial.print(F("red="));
//       Serial.print(red_Led, DEC);
//       Serial.print(F(", ir="));
//       Serial.print(aun_ir_buffer[i], DEC);
//       
//       Serial.print(F(", HR="));
//       Serial.print(n_heart_rate, DEC);
//       
//       Serial.print(F(", HRvalid="));
//       Serial.println(ch_hr_valid, DEC);
       
     }
     maxim_heart_rate(aun_ir_buffer, BUFFER_LENGTH, &n_heart_rate, &ch_hr_valid); 
     
       /////////////////////////// POST FILTERING ////////////////
       if ((n_heart_rate < minHR) || (n_heart_rate > maxHR)){
          n_heart_rate = defaultHR;
        }
        else{
          maxHR = n_heart_rate + MAX_DEVIATION;
          minHR = n_heart_rate - MAX_DEVIATION;
          }
       ////////////////////////////////////////////////////////////
       client.println(ID+n_heart_rate);
       
    } //while(persona)
   } // if(persona)
   else{ //si no hi ha persona...

      Serial.println(persona);
      Serial.println(red_Led);
      for (i=0; i<4;i++){
        while(digitalRead(9)==1);  //wait until the interrupt pin detects new data from sensor
        maxim_max30102_read_fifo((&red_Led), (&dummy));  //read from MAX30102 FIFO  
        Serial.println(red_Led);
      }

      if(red_Led >= HUMAN_YES){
        maxHR = 120;  
        minHR = 50;
        persona = true;
        client.println("Y0");
        //client.println("R300"); //no estressar el motor mentre calcula els 4s...
        //fer algo amb el motor ----> usar delays?
      }
      if (!persona){
          maxim_max30102_write_reg(REG_MODE_CONFIG,131);
          delay(1500);
          maxim_max30102_write_reg(REG_MODE_CONFIG,3);
          delay(500);
      }
   }
 }
}
