#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>

int enc = 0;

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;

float Ki = 0;
float Kp = 0.0438;//0.04352
float Kd = 0.099; // 0.0992

float KiR = 0;
float KpR = 0.0168;//0.0392
float KdR = 0.0839; // 0.097

#define BLYNK_PRINT Serial

/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID   "TMPLa2VAm_FV"


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "k0uTh2IJ18o7CHMXs2DlYBY8jnuJl5To";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Renzzo";
char pass[] = "Renzo753159456";

// BLYNK_WRITE(V0)
// {
//   if(param.asInt()==1){
//     Serial.println("recebi");
//     // analogWrite(0, velesq);
//     // analogWrite(0, veldir);
//   }
//   else{
//     Serial.println("recebi 2");
//   }
// }

void ler_sensores()
{

  uint16_t sArraychannels[sArray.getSensorCount()];
  erro_sensores = sArray.readLineWhite(sArraychannels) - 3500;
  erro_f = -1 * erro_sensores;
 
}

void calcula_PID()
{
  
  P = erro_f;
  D = erro_f - erro_anterior;
  PID = (Kp * P) + (Kd * D);
  erro_anterior = erro_f;
}

void controle_motores(float vel_A, float vel_B)
{
  
  velesq = vel_A + PID;
  veldir = vel_B - PID;
  if (velesq < 15)
  {
    velesq = 15;
  }
  
  if (veldir <15)
  {
    veldir = 15;
  }
  digitalWrite(in_dir1, HIGH);
  digitalWrite(in_dir2, LOW);
  analogWrite(pwmA, velesq);

  digitalWrite(in_esq1, LOW);
  digitalWrite(in_esq2, HIGH);
  analogWrite(pwmB, veldir);
}

void calcula_PID_R()
{
  
  PR = erro_f;
  DR = erro_f - erro_anterior;
  PIDR = (KpR * PR) + (KdR * DR);
  erro_anterior = erro_f;
}

void controle_motores_R()
{
  float vel_AR = 255;
  float vel_BR = 255;
  velesqR = vel_AR + PIDR;
  veldirR = vel_BR- PIDR;
  if (velesqR < 15)
  {
    velesqR = 15;
  }
  
  if (veldirR <15)
  {
    veldirR = 15;
  }
  digitalWrite(in_dir1, HIGH);
  digitalWrite(in_dir2, LOW);
  analogWrite(pwmA, velesqR);

  digitalWrite(in_esq1, LOW);
  digitalWrite(in_esq2, HIGH);
  analogWrite(pwmB, veldirR);
}



int calculate_rpm()
{

  enc_esq_pul = encoder.getCount() - pul_prev_eq;   // delta s
  enc_dir_pul = encoder2.getCount() - pul_prev_dir; // delta s

  pul_prev_eq = encoder.getCount();
  pul_prev_dir = encoder2.getCount();

  // Pulses multiplied by 100 because interrupt every 10milliseconds (Convert to pulses per second)
  // Divide by pulses per turn of output shaft.
  // Multiply by 60 to go from seconds to minutes
  

  enc = (enc_esq_pul + enc_dir_pul) /2;
  Serial.println(enc);
  return enc;
  
}
bool ler_sens_lat()
{
  #define tempoDebounce 200

  bool estadoSLatEsq;
  static bool estadoSLatEsqAnt;
  static bool estadoRet = true;
  static unsigned long delaySenLat = 0;
  int x = 0;
  int y = 0;
  if((millis() - delaySenLat)> tempoDebounce){
    x = analogRead(s_lat_esq);
    y = analogRead(s_lat_dir);
    // if(x < 100  && y <100){
    //   estadoSLatEsq = false;     
    // }
    if(x < 150){
      estadoSLatEsq = true;     
    }
    else{
      estadoSLatEsq = false;
    }
    
    
    
    if(estadoSLatEsq && (estadoSLatEsq != estadoSLatEsqAnt)){
      estadoRet = !estadoRet;
      delaySenLat = millis();
    }
    estadoSLatEsqAnt = estadoSLatEsq;
    
  }
  return estadoSLatEsq;
}


/*bool ler_sens_lat_Dir()
{
  #define tempoDebounce2 200

  bool estadoSLatDir;
  static bool estadoSLatDirAnt;
  static bool estadoRetDir = true;
  static unsigned long delaySenLatDir = 0;
  int x = 0;
  int y = 0;
  if((millis() - delaySenLatDir)> tempoDebounce2){
    y = analogRead(s_lat_dir);
    // if(x < 100  && y <100){
    //   estadoSLatEsq = false;     
    // }
    if(y < 150){
      estadoSLatDir = true;     
    }
    else{
      estadoSLatDir = false;
    }
    
    
    
    if(estadoSLatDir && (estadoSLatDir != estadoSLatDirAnt)){
      estadoRetDir = !estadoRetDir;
      delaySenLatDir = millis();
    }
    estadoSLatDirAnt = estadoSLatDir;
    
  }
  return estadoSLatDir;
}

*/

void controle_com_mapeamento(int encVal){
  if(encVal > 1800 && encVal < 8500){
      calcula_PID_R();
      controle_motores_R();

    }

    else if(encVal > 25000 && encVal < 29000){
      calcula_PID_R();
      controle_motores_R();

    }else if(encVal > 41500 && encVal < 45000){
      calcula_PID_R();
      controle_motores_R();

    }
    else if(encVal > 78000 && encVal < 95500){
      calcula_PID_R();
      controle_motores_R();

    }
    else if(encVal > 97500 && encVal < 105000){
      calcula_PID_R();
      controle_motores_R();

    }
    else if(encVal > 109000 && encVal < 130000){
      calcula_PID_R();
      controle_motores_R();

    }
    else if(encVal > 132000 && encVal < 140000){
      calcula_PID_R();
      controle_motores_R();

    }
    else if(encVal > 163800 && encVal < 170000){
      calcula_PID_R();
      controle_motores_R();

    }  
    else if(encVal > 173500 && encVal < 182000){
      calcula_PID_R();
      controle_motores_R();

    }  
    else if(encVal > 187000 && encVal < 194000){
      calcula_PID_R();
      controle_motores_R();

    }  
    else if(encVal > 203500 && encVal < 240000){
      calcula_PID_R();
      controle_motores_R();

    }  
    else if(encVal > 266000){
      digitalWrite(stby, LOW);

    }
    else{
      calcula_PID();
      controle_motores(140,140);
    } 
  }
void controle_sem_mapeamento(){

        calcula_PID();
        controle_motores(145,145);
}

int v = 0;
void mapeamento(){
  digitalWrite(led, LOW);
  if(ler_sens_lat() == true){  
        
          v = (v+1);
          Serial.print("Marca ");
          Serial.print(v);
           Serial.print(": ");
          Serial.print((encoder.getCount() + encoder2.getCount())/2);
          //Serial.println(timer_in);

        
      digitalWrite(led, HIGH);  
       
     }
}


void setup()
{
  Serial.begin(9600);
  
 

  pinMode(in_dir1, OUTPUT);
  pinMode(in_dir2, OUTPUT);
  pinMode(in_esq1, OUTPUT);
  pinMode(in_esq2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(s_lat_esq, INPUT);
  pinMode(s_lat_dir, INPUT);
  pinMode(buzzer, OUTPUT);

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_B, enc_eq_A);
  encoder2.attachFullQuad(enc_dir_A, enc_dir_B);

  digitalWrite(stby, HIGH);

  encoder.clearCount();
  encoder2.clearCount();

  sArray.setTypeMCP3008();
  sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
  sArray.setSamplesPerSensor(5);

  for (uint16_t i = 0; i < 300; i++)
  {
    sArray.calibrate();
    delay(20);
  }

  //Blynk.run();
  
}
bool bly = false;

int d = 0;

void loop()
{
 
  digitalWrite(led, LOW);
  digitalWrite(buzzer, LOW);
  timer_in = millis();
   

          
  if(ler_sens_lat() == true){  
        if(timer_in - timer_prev3 >= 10){
          v = (v+1);
          Serial.print("Marca ");
          Serial.print(v);
          Serial.print(": ");
          Serial.println((encoder.getCount() + encoder2.getCount())/2);
          //Serial.println(timer_in);

        }
      digitalWrite(led, HIGH); 
      digitalWrite(buzzer, HIGH);
       timer_prev3 = timer_in;
     }
    
    if(timer_in - timer_prev >= 10){
      ler_sensores();
      controle_sem_mapeamento();
      float encVal = ((encoder.getCount() + encoder2.getCount())/2);
      if(encVal > 270000){
        digitalWrite(stby, LOW);
      }
    }



 
}