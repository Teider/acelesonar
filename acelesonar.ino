#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define NUM_MEDIAS_OFFSET 100
#define K_MEDIA_MOVEL     10

#define WEIGHT_ACCEL_COMP_FILTER  0.2

#define trigPin 6
#define echoPin1 7   
#define echoPin2 8
#define echoPin3 9
#define echoPin4 10
#define echoPin5 11
#define echoPin6 12
#define LED_PIN 13

MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

double aRoll_offset = 0, aPitch_offset;
long gx_rate_offset = 0, gy_rate_offset = 0, gz_rate_offset = 0;

double aRoll_read[K_MEDIA_MOVEL], aPitch_read[K_MEDIA_MOVEL];
double gRoll_rate_read[K_MEDIA_MOVEL], gPitch_rate_read[K_MEDIA_MOVEL], gYaw_rate_read[K_MEDIA_MOVEL];

int current_read_counter = 0;

double aRoll_avg = 0.0, aPitch_avg = 0.0;
double gRoll_rate_avg = 0.0, gPitch_rate_avg = 0.0, gYaw_rate_avg = 0.0;

double gYaw_sum = 0.0;
double compRoll = 0.0, compPitch = 0.0;

long rollAngle = 0, pitchAngle = 0, yawAngle = 0;

double current_time = 0.0;
double last_time = 0.0;
double delta_t = 0.0;

unsigned int duration;
unsigned int distancia;

char ler;

SoftwareSerial sensorSerial(4, 5);

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//    Serial.begin(9600);

    sensorSerial.begin(9600);

    accelgyro.initialize();
    
   pinMode(echoPin1, INPUT);   
   pinMode(echoPin2, INPUT);
   pinMode(echoPin3, INPUT);
   pinMode(echoPin4, INPUT);
   pinMode(echoPin5, INPUT);
   pinMode(echoPin6, INPUT);
   pinMode(trigPin, OUTPUT); 
   pinMode(LED_PIN, OUTPUT);
   
   delay(10);
   
   for (int i = 0; i < NUM_MEDIAS_OFFSET; i++) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      aRoll_read[0] =  atan2(ay, az);
      aRoll_read[0] *= (180 / PI);

      aPitch_read[0] =  atan2(ax, az);
      aPitch_read[0] *= (180 / PI);
      
      aRoll_offset += aRoll_read[0];
      aPitch_offset += aPitch_read[0];
      
      gx_rate_offset += gx;
      gy_rate_offset += gy;
      gz_rate_offset += gz;
      
      delay(10);
      
    }
    
    aRoll_offset /= NUM_MEDIAS_OFFSET;
    aPitch_offset /= NUM_MEDIAS_OFFSET;
    gx_rate_offset /= NUM_MEDIAS_OFFSET;
    gy_rate_offset /= NUM_MEDIAS_OFFSET;
    gz_rate_offset /= NUM_MEDIAS_OFFSET;
    
    current_time = micros() / 1000000.0;
    last_time = micros() / 1000000.0;
    delta_t = 0.0;
    
    for (int i = 0; i < K_MEDIA_MOVEL; i++) {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
      current_time = micros() / 1000000.0;
      delta_t = current_time - last_time;
      last_time = current_time;
        
      aRoll_read[i] =  atan2(ay, az);
      aRoll_read[i] *= (180 / PI);
      aRoll_read[i] -= aRoll_offset;
        
      aPitch_read[i] =  atan2(ax, az);
      aPitch_read[i] *= (180 / PI);
      aPitch_read[i] -= aPitch_offset;
        
      gx -= gx_rate_offset;
      gy -= gy_rate_offset;
      gz -= gz_rate_offset;
        
      gRoll_rate_read[i] = (gx / 131.0) * delta_t;
      gPitch_rate_read[i] = (gy / 131.0) * delta_t;
      gYaw_rate_read[i] = (gz / 131.0) * delta_t;
      
      aRoll_avg += (aRoll_read[i] / K_MEDIA_MOVEL);
      aPitch_avg += (aPitch_read[i] / K_MEDIA_MOVEL);
      
      gRoll_rate_avg += (gRoll_rate_read[i] / K_MEDIA_MOVEL);
      gPitch_rate_avg += (gPitch_rate_read[i] / K_MEDIA_MOVEL);
      gYaw_rate_avg += (gYaw_rate_read[i] / K_MEDIA_MOVEL);
      
      delay(10);
    }
   
   
   //Serial.println("pronto");
   //sensorSerial.write(0x1C);
}

void loop() {
  
  if(sensorSerial.available()>0) {
/*
    Serial.print ("numero de bytes pra ler: ");
    Serial.println (sensorSerial.available());
*/  
    ler = sensorSerial.read();
//    Serial.print("byte lido: ");
//    Serial.println(ler,HEX);
    if (ler == 0x18){
/*        
        Serial.println("Enviando dados acelerometro");
        
        Serial.print("Medida do aRoll: ");
        Serial.println(aRoll_avg, 8);
          
        Serial.print("Medida do compRoll: ");
        Serial.println(compRoll, 8);
        
        Serial.print("Medida do rollAngle: ");
        Serial.println(rollAngle, DEC);
        
        Serial.println();
        
        Serial.print("Medida do aPitch: ");
        Serial.println(aPitch_avg, 8);
          
        Serial.print("Medida do compPitch: ");
        Serial.println(compPitch, 8);
        
        Serial.print("Medida do pitchAngle: ");
        Serial.println(pitchAngle, DEC);
        
        Serial.println("-----//-----//-----");
        
        Serial.println(highByte(rollAngle),HEX);
        Serial.println(lowByte(rollAngle),HEX);

        Serial.println(highByte(pitchAngle),HEX);
        Serial.println(lowByte(pitchAngle),HEX);

        Serial.println(highByte(yawAngle),HEX);
        Serial.println(lowByte(yawAngle),HEX);
*/        
        sensorSerial.write(0x19);

        sensorSerial.write(highByte(rollAngle));
        sensorSerial.write(lowByte(rollAngle));

        sensorSerial.write(highByte(pitchAngle));
        sensorSerial.write(lowByte(pitchAngle));

        sensorSerial.write(highByte(yawAngle));
        sensorSerial.write(lowByte(yawAngle));
        
    }
    else if (ler == 0x1b){
      
//      Serial.println("Pediu dados sonares");        

      ler = sensorSerial.read();
      sensorSerial.write(0x1a);
      sensorSerial.write(ler);
      
      switch (ler) {
        case 0x01:
//              Serial.println("Enviando sonar1");        
          //faz sensor1
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin1,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
              
        case 0x02:
//              Serial.println("Enviando sonar2");        

          //faz sensor2
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin2,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
       
        case 0x03:
               
//               Serial.println("Enviando sonar3");        

          //faz sensor3
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin3,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
              
          case 0x04:
//                            Serial.println("Enviando sonar4");        

          //faz sensor4
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin4,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
              
          case 0x05:
//                            Serial.println("Enviando sonar5");        

          //faz sensor5
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin5,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
              
          case 0x06:
//                            Serial.println("Enviando sonar6");        

          //faz sensor6
              digitalWrite(trigPin, LOW);  
              delayMicroseconds(2);  
              digitalWrite(trigPin, HIGH);  
              delayMicroseconds(10);  
              digitalWrite(trigPin, LOW);  
              duration = pulseIn(echoPin6,HIGH);  
              distancia = duration /29 /2 ;  
              sensorSerial.write(highByte(distancia));
              sensorSerial.write(lowByte(distancia));
              break;
              
              delay(1);
              
      }
    }
//      else {Serial.println ("byte recebido nao pede nenhum sensor");}
    
    
    
  } else {
      
      aRoll_avg -= (aRoll_read[current_read_counter] / K_MEDIA_MOVEL);
      aPitch_avg -= (aPitch_read[current_read_counter] / K_MEDIA_MOVEL);
      
      gRoll_rate_avg -= (gRoll_rate_read[current_read_counter] / K_MEDIA_MOVEL);
      gPitch_rate_avg -= (gPitch_rate_read[current_read_counter] / K_MEDIA_MOVEL);
      gYaw_rate_avg -= (gYaw_rate_read[current_read_counter] / K_MEDIA_MOVEL);
      
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
      current_time = micros() / 1000000.0;
      delta_t = current_time - last_time;
      last_time = current_time;
        
      aRoll_read[current_read_counter] =  atan2(ay, az);
      aRoll_read[current_read_counter] *= (180 / PI);
      aRoll_read[current_read_counter] -= aRoll_offset;
        
      aPitch_read[current_read_counter] =  atan2(ax, az);
      aPitch_read[current_read_counter] *= (180 / PI);
      aPitch_read[current_read_counter] -= aPitch_offset;
      
      aRoll_avg += (aRoll_read[current_read_counter] / K_MEDIA_MOVEL);
      aPitch_avg += (aPitch_read[current_read_counter] / K_MEDIA_MOVEL);
      
      gx -= gx_rate_offset;
      gy -= gy_rate_offset;
      gz -= gz_rate_offset;
        
      gRoll_rate_read[current_read_counter] = (gx / 131.0) * delta_t;
      gPitch_rate_read[current_read_counter] = (gy / 131.0) * delta_t;
      gYaw_rate_read[current_read_counter] = (gz / 131.0) * delta_t;
      
      gRoll_rate_avg += (gRoll_rate_read[current_read_counter] / K_MEDIA_MOVEL);
      gPitch_rate_avg += (gPitch_rate_read[current_read_counter] / K_MEDIA_MOVEL);
      gYaw_rate_avg += (gYaw_rate_read[current_read_counter] / K_MEDIA_MOVEL);
        
      compRoll += gRoll_rate_avg;
      compRoll = (1 - WEIGHT_ACCEL_COMP_FILTER) * compRoll + WEIGHT_ACCEL_COMP_FILTER * aRoll_avg;
        
      compPitch += gPitch_rate_avg;
      compPitch = (1 - WEIGHT_ACCEL_COMP_FILTER) * compPitch + WEIGHT_ACCEL_COMP_FILTER * aPitch_avg;
                    
      rollAngle = (60 * compRoll);
      pitchAngle = (60 * compPitch);
      yawAngle = 0;
      
      rollAngle += 10800;
      pitchAngle += 10800;
      yawAngle += 10800;
      
      current_read_counter++;
      if (current_read_counter == K_MEDIA_MOVEL) {
        current_read_counter = 0;
      }
      
  }

  delay(10);

}
