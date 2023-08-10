#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

int enc = 0;

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;
BluetoothSerial SerialBT;

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 2

// Declare our NeoPixel strip object:
Adafruit_NeoPixel led_stip(LED_COUNT, led, NEO_GRB + NEO_KHZ800);

float Ki = 0;
float Kp = 0.0445;//0.04352
float Kd = 0.340; // 0.0992

float KiR = 0;
float KpR = 0.035;//0.0392
float KdR = 0.0899; // 0.097

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
  //Serial.println("controle motor");
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
void controle_motores_R(float vel_AR, float vel_BR){
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

bool ler_sens_lat_esq()
{
  #define tempoDebounce 20

  bool estadoSLatEsq;
  static bool estadoSLatEsqAnt;
  static bool estadoRet = true;
  static unsigned long delaySenLat = 0;
  int x = 0;
  if((millis() - delaySenLat)> tempoDebounce){
    x = analogRead(s_lat_esq);

    if(x < 200){
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


bool ler_sens_lat_dir()
{
  #define tempoDebounce2 20

  bool estadoSLatDir;
  static bool estadoSLatDirAnt;
  static bool estadoRetDir = true;
  static unsigned long delaySenLatDir = 0;
  int y = 0;
  if((millis() - delaySenLatDir)> tempoDebounce2){
    y = analogRead(s_lat_dir);
    
    if(y < 200){
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

void controle_sem_mapeamento(){

        calcula_PID();
        controle_motores(100,100);
}

//##############################################################################################
//################ INICIO: NOVA PROPOSTA DE CONTROLE COM MAPEAMENTO ############################
struct Range {
    int minValue;
    int maxValue;
    int leftMotorSpeed;
    int rightMotorSpeed;
};

const int numRanges = 8;  // Número de intervalos

Range ranges[numRanges] = {
    {0, 24000, 135, 135},         // intervalo 1
    {99000, 103600, 245, 245},    // intervalo 2
    {109850, 114600, 245, 245},   // intervalo 3
    {119600, 137000, 255, 255},   // intervalo 3
    {144700, 149000, 245, 245},   // intervalo 4
    {178000, 207900, 240, 240},   // intervalo 5
    {221000, 280000, 255, 255},   // intervalo 7
    {275000, INT_MAX, 120, 120}   // final
};

void calculaEControlePID_R(int leftSpeed, int rightSpeed) { //Implementar parametro de tipo de PID; reta, curva, curva longa
    calcula_PID_R();
    controle_motores_R(leftSpeed, rightSpeed);
}

void controle_com_mapeamento2(int encVal) {

    for (int i = 0; i < numRanges; ++i) {
        if (encVal > ranges[i].minValue && encVal < ranges[i].maxValue) {

            calculaEControlePID_R(ranges[i].leftMotorSpeed, ranges[i].rightMotorSpeed);

            if (i > numRanges-1) {
                digitalWrite(stby, LOW);
            }
            return;
        }
    }
    // Se nenhum intervalo for correspondido, executar ação padrão
    controle_sem_mapeamento();
}

//################ FIM: NOVA PROPOSTA DE CONTROLE COM MAPEAMENTO ###############################
//##############################################################################################


void controle_com_mapeamento(int encVal){
  digitalWrite(buzzer, LOW);
  if(encVal > 0 && encVal < 24000){
        calcula_PID();
        controle_motores(135,135);
      }
      else if(encVal > 99000 && encVal < 103600){ //diagonal
        digitalWrite(buzzer, HIGH);
        calcula_PID_R();
        controle_motores_R(245, 245);
      }      else if(encVal > 109850 && encVal < 114600){ //antes reta media
        digitalWrite(buzzer, HIGH);
        calcula_PID_R();
        controle_motores_R(245, 245);

      }else if(encVal > 119600 && encVal < 137000){ //reta media
        digitalWrite(buzzer, HIGH);
        calcula_PID_R();
        controle_motores_R(255, 255);
      }
      else if(encVal > 144700 && encVal < 149000){ //reta vertical
        digitalWrite(buzzer, HIGH);

        calcula_PID_R();
        controle_motores_R(245, 245);
      }  
      else if(encVal > 178000 && encVal < 183300){ //reta 1 de 3
          digitalWrite(buzzer, HIGH);

          calcula_PID_R();
          controle_motores_R(240, 240);
      }
      else if(encVal > 190000 && encVal < 195400){ //reta 2 de 3
          digitalWrite(buzzer, HIGH);

          calcula_PID_R();
          controle_motores_R(240, 240);
      }
      else if(encVal > 202200 && encVal < 207900){ //reta 3 de 3
        digitalWrite(buzzer, HIGH);

        calcula_PID_R();
        controle_motores_R(240, 240);
      }
      else if(encVal > 221000 && encVal < 251000){ //retona
        digitalWrite(buzzer, HIGH);

        calcula_PID_R();
        controle_motores_R(255, 255);
      } 
      else if(encVal > 250800 && encVal < 280000){ //final
        calcula_PID();
        controle_motores(120,120);
      }
      else if(encVal > 275000){
        digitalWrite(stby, LOW);
      }
      else{
        controle_sem_mapeamento();
      }
  }

int v = 0;

void mapeamento(){
  timer_in = millis();

  digitalWrite(buzzer, LOW);
          
  if(ler_sens_lat_dir() == true){  
        if(timer_in - timer_prev3 >= 10){
          v = (v+1);
          Serial.print("Marca ");
          Serial.print(v);
          Serial.print(": ");
          Serial.println((encoder.getCount() + encoder2.getCount())/2);
          //Serial.println(timer_in);

        }
      digitalWrite(buzzer, HIGH);
       timer_prev3 = timer_in;
     }
    
}
void setup()
{
  Serial.begin(115200);

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

  led_stip.begin();
  SerialBT.begin("Semreh"); //Bluetooth device name

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
    led_stip.setPixelColor(1, 0, 255, 0);
    led_stip.show();
    sArray.calibrate();
    delay(20);
    led_stip.setPixelColor(1, 0, 0, 0);
    led_stip.show();
  }
  
}
bool bly = false;

void loop()
{
  led_stip.setPixelColor(0, 255, 0, 0);
  led_stip.show();
  ler_sensores();
  int encVal = ((encoder.getCount() + encoder2.getCount())/2);
  SerialBT.println(encVal);
  controle_sem_mapeamento();
  digitalWrite(buzzer, ler_sens_lat_esq());
  digitalWrite(buzzer, ler_sens_lat_dir());
}

