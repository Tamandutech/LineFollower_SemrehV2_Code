#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;
BluetoothSerial SerialBT;
Adafruit_NeoPixel led_stip(LED_COUNT, led, NEO_GRB + NEO_KHZ800); // Declare our NeoPixel strip object

const uint32_t R = led_stip.Color(255,0,0);
const uint32_t G = led_stip.Color(0,255,0);
const uint32_t B = led_stip.Color(0,0,255);
const uint32_t W = led_stip.Color(255,255,255);

u_int64_t tempo;

bool FirstTimeOnSwitchCase = true;

long int calculate_rpm_esq();
long int calculate_rpm_dir();

void ler_sensores()
{
  uint16_t sArraychannels[sArray.getSensorCount()];
  erro_sensores = sArray.readLineWhite(sArraychannels) - 3500;
  erro_f = -1 * erro_sensores;
}

void calcula_PID(float KpParam, float KdParam)
{
  P = erro_f;
  D = erro_f - erro_anterior;
  PID = (KpParam * P) + (KdParam * D);
  erro_anterior = erro_f;
}

void calcula_PID_rot(float KpParamRot , float KdParamRot , float KiParamrot){
  erro_f_rot = (calculate_rpm_esq() - calculate_rpm_dir());
  P_rot = erro_f_rot;
  D_rot = erro_f_rot - erro_anterior_rot;
  I_rot += erro_f_rot;
  PIDrot =(KpParamRot*P_rot)+(KdParamRot*D_rot)+(KiParamRot*I_rot);
  erro_anterior_rot = erro_f_rot;
  
}

float curva_acel(float pwm_goal)
{

  if (pwm_goal > last_pwm)
  {
    last_pwm += 5;
    run_pwm = last_pwm;
  }
  else
  {
    run_pwm = pwm_goal;
  }
  return run_pwm;
}

void controle_motores(float motorPWM)
{
  velesq = curva_acel(motorPWM) + PID;
  veldir = curva_acel(motorPWM) - PID;

  if(veldir >= 0)
  {
    if(veldir > MAX_PWM) veldir = MAX_PWM;
    digitalWrite(in_dir1,LOW);
    digitalWrite(in_dir2,HIGH);
  }
  else
  {
    veldir = (-1) * veldir;
    digitalWrite(in_dir1,HIGH);
    digitalWrite(in_dir2,LOW);
  }
  analogWrite(PWM_RIGHT,veldir);

  if(velesq >= 0)
  {
    if (velesq > MAX_PWM) velesq = MAX_PWM;
    digitalWrite(in_esq1,LOW);
    digitalWrite(in_esq2,HIGH);
  }
  else
  {
    velesq = (-1) * velesq;
    digitalWrite(in_esq1,HIGH);
    digitalWrite(in_esq2,LOW);
  }
  analogWrite(PWM_LEFT,velesq);
}

void controle_motores_rot(float motorPWM)
{
  velesqrot = motorPWM + PIDrot;
  veldirrot = motorPWM - PIDrot;

  if(veldirrot >= 0)
  {
    if(veldirrot > MAX_PWM) veldirrot = MAX_PWM;
    digitalWrite(in_dir1,LOW);
    digitalWrite(in_dir2,HIGH);
  }
  else
  {
    veldirrot = (-1) * veldirrot;
    digitalWrite(in_dir1,HIGH);
    digitalWrite(in_dir2,LOW);
  }
  analogWrite(PWM_RIGHT,veldirrot);

  if(velesqrot >= 0)
  {
    if (velesqrot > MAX_PWM) velesqrot = MAX_PWM;
    digitalWrite(in_esq1,LOW);
    digitalWrite(in_esq2,HIGH);
  }
  else
  {
    velesqrot = (-1) * velesqrot;
    digitalWrite(in_esq1,HIGH);
    digitalWrite(in_esq2,LOW);
  }
  analogWrite(PWM_LEFT,velesqrot);
}

struct MotorControlData {
    float encoderValueBegin;
    float encoderValueEnd;
    int motorPWM;
};

std::vector<MotorControlData> trackMap = {
  //{encoderValueBegin, encoderValueEnd, motorPWM},
    {0, 220, 120},
    {223.5,	866.5,	110},
    {866.5,	1061.6,	170},
    {1591.5,	1909.0,	108},
    {1909,	2059.4,	170},
    {2587.5,	2850.0,	109},
    {2850,	2875.4,	119},
    {3300,	5085.5,	210}, //bolona
    {5448.5,	6644.0,	169},
    {6769,	6991.0,	126},
    {6991,	6786.5,	255},
    {7180.5,	7378.3,	159},
    {7497.5,	7939.5,	115},
    {7939.5,	8262.9,	202},
    {8571.5,	9471.0,	99},
    {9900,	11300.0,	240}, //reta cruzamento
    {11987,	13300.5,	115},
    //{14950,	15956.0,	240},
    {15956,	18390.5,	130},
    {18800,	19227.5,	117},
    {19700.5,	20926.6,	240},
    {21434,	21833.5,	118},
    {22400.5,	25500/*24977.7*/,	240},
    {26300, 999999, 0}
};

void motorControlWithMap(int encVal)
{
  bool not_mapped = true;
  for (const auto& MotorControlData : trackMap)
  {
    float firstEncoderValue = MotorControlData.encoderValueBegin;
    float secondEncoderValue = MotorControlData.encoderValueEnd;
    int pwmDeliveredToMotors = MotorControlData.motorPWM;

    if (encVal >= firstEncoderValue && encVal <= secondEncoderValue)
    {
      if(pwmDeliveredToMotors>=220)
      {
        calcula_PID(KpR, KdR);
        controle_motores(pwmDeliveredToMotors);
      }
      
      else
      { 
        calcula_PID(Kp, Kd);
        controle_motores(pwmDeliveredToMotors);
      }
      not_mapped = false;
      //return;
    }
  }
  if(not_mapped){
    calcula_PID(Kp, Kd);
    controle_motores(100);
  }
}

void controle_com_mapeamento(int encVal)
{
  if(encVal<200) {calcula_PID(Kp,Kd); controle_motores(120);} //começo de pista lateral direito até primeira marcaçao
  else if (encVal >= 223 && encVal <= 866) {calcula_PID(Kp,Kd); controle_motores(110);}
  else if (encVal >= 930 && encVal <= 1061) {calcula_PID(Kp,Kd); controle_motores(255);}
  else if (encVal >= 1600 && encVal <= 1900) {calcula_PID(Kp,Kd); controle_motores(115);}
  else if (encVal >= 1940 && encVal <= 2000) {calcula_PID(Kp,Kd); controle_motores(255);}
  else if (encVal >= 2600 && encVal <= 2700) {calcula_PID(Kp,Kd); controle_motores(115);}
  else if (encVal >= 2750 && encVal <= 2910) {calcula_PID(Kp,Kd); controle_motores(200);}
  else if (encVal >= 3407 && encVal <= 5080) {calcula_PID(Kp,Kd); controle_motores(210);} //bolona
  else if (encVal >= 5450 && encVal <= 6600) {calcula_PID(Kp,Kd); controle_motores(169);}
  else if (encVal >= 6800 && encVal <= 6900) {calcula_PID(Kp,Kd); controle_motores(126);}
  else if (encVal >= 7000 && encVal <= 7300) {calcula_PID(Kp,Kd); controle_motores(150);}
  else if (encVal >= 7950 && encVal <= 8200) {calcula_PID(Kp,Kd); controle_motores(200);}
  else if (encVal >= 9750 && encVal <= 11300) {calcula_PID(KpR,KdR); controle_motores(240);} //reta cruzamen
  else if (encVal >= 12300 && encVal <= 13300) {calcula_PID(KpR,KdR); controle_motores(150);} //pós curz
  else if (encVal >= 13900 && encVal <= 14800) {calcula_PID(KpR,KdR); controle_motores(120);} //180 graus
  else if (encVal >= 14900 && encVal <= 15800) {calcula_PID(KpR,KdR); controle_motores(240);} //reta antes do zig
  else if (encVal >= 15801 && encVal <= 18300) {digitalWrite(in_dir1,LOW);digitalWrite(in_dir2,HIGH);analogWrite(PWM_RIGHT,200);digitalWrite(in_esq1,LOW);digitalWrite(in_esq2,HIGH);analogWrite(PWM_LEFT,185);} //zig PARA IR PRA DIREITA AUMENTAR PWM ESQUERDA (AINDA ESTÁ INDO PRA A ESQUERDA)
  // else if (encVal >= 19800 && encVal <= 21000) {calcula_PID(KpR,KdR); controle_motores(250);}
  // else if (encVal >= 22000 && encVal <= 25000) {calcula_PID(KpR,KdR); controle_motores(250);}
  else if (encVal>26500) {analogWrite(PWM_LEFT,0); analogWrite(PWM_RIGHT,0); analogWrite(PROPELLER_PIN, 0);}
  else{calcula_PID(Kp,Kd); controle_motores(110); led_stip.setPixelColor(0,G);}
}

void ler_laterais(void *parameter){
  static bool readingWhiteLeft;
  static bool readingWhiteRight; // variável para que a marcação seja lida apenas uma vez
  static bool firstTimeRight = false; // variável para evitar leitura de lateral direito no meio da pista
  while(true)
  {
    accumLateralEsq[countLateral] = analogRead(s_lat_esq);
    accumLateralDir[countLateral] = analogRead(s_lat_dir);
    countLateral++;

    for (int i=0; i<MED_TAMANHO; i++) 
    {
      medLateralDir+= accumLateralDir[i];
      medLateralEsq+= accumLateralEsq[i];
    }

    medLateralDir = medLateralDir / MED_TAMANHO;
    medLateralEsq = medLateralEsq / MED_TAMANHO;
    
    if(medLateralEsq < 3000 || medLateralDir < 3000) //lê marcação esquerdo ou direito esq3100 dir3700
    {
      if(medLateralEsq < 2000 && medLateralDir > 3500 && readingWhiteLeft == false) //lê apenas marcação esquerda
      {
        SerialBT.print(';'); 
        SerialBT.print(encoder.getCount()); 
        SerialBT.print(';'); 
        SerialBT.print(encoder2.getCount()); 
        SerialBT.print(';'); 
        SerialBT.println((encoder.getCount() + encoder2.getCount())/2);
        led_stip.setPixelColor(1, 0, 0, 255);
        led_stip.show();
        // digitalWrite(buzzer, HIGH);  // Ligar o buzzer
        // vTaskDelay(pdMS_TO_TICKS(40));  // Manter o buzzer ligado por 100ms
        // digitalWrite(buzzer, LOW);   // Desligar o buzzer
        led_stip.setPixelColor(1, 0, 0, 0);
        led_stip.show();
        readingWhiteLeft = true;
        digitalWrite(buzzer, HIGH);
      }

      else if(medLateralEsq > 3500 && medLateralDir < 2000 && readingWhiteRight == false && firstTimeRight == false) //lê apenas marcação direita
      {
        tempo = millis();
        SerialBT.println("INICIO/FIM");
        encoder2.clearCount();
        encoder.clearCount();
        firstTimeRight = true;
        readingWhiteRight = true;
        digitalWrite(buzzer, HIGH);
        // vTaskDelay(pdMS_TO_TICKS(40));  // Manter o buzzer ligado por 100ms
        // digitalWrite(buzzer, LOW);   // Desligar o buzzer
      }
    }

    else
    {
      readingWhiteLeft = false;
      readingWhiteRight = false;
      digitalWrite(buzzer, LOW);
    }
    medLateralDir = 0;
    medLateralEsq = 0;

    countLateral = countLateral % MED_TAMANHO;
    vTaskDelay(pdMS_TO_TICKS(8));  // Pausa de 5ms entre as verificações
  }
}

char lastReceivedChar = '3'; // = Abort

void bluetoothRead()
{
  if(SerialBT.available()) 
  {
    char incomingChar = (char)SerialBT.read();
    while (SerialBT.available())
    {
      SerialBT.read();
    }
    lastReceivedChar = incomingChar;
  }
}

void callRobotTask(char status)
{
  switch(status)
  {
  case '1': //Map
    ler_sensores();
    calcula_PID(Kp,Kd);
    controle_motores(MAPPING_PWM);
  break;

  case '2': //Run with track map
    ler_sensores();
    motorControlWithMap((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '3': //Abort
    analogWrite(PWM_LEFT,0);
    analogWrite(PWM_RIGHT,0);
    analogWrite(PROPELLER_PIN, 0);
  break;

  case '4': //Pausa locomoção
    analogWrite(PWM_LEFT,0);
    analogWrite(PWM_RIGHT,0);
  break;

  case '6': //Propeller
    analogWrite(PROPELLER_PIN,PROPELLER_PWM);
  break;
  }
}

long int calculate_rpm_esq(){
  Serial.print(enc_esq_pul);
  Serial.print(", ");
  Serial.println(pul_prev_esq);
  enc_esq_pul = encoder.getCount() - pul_prev_esq;
  pul_prev_esq = encoder.getCount();

  return enc_esq_pul;
}

long int calculate_rpm_dir(){
  Serial.print(enc_dir_pul);
  Serial.print(", ");
  Serial.println(pul_prev_dir);
  enc_dir_pul =  encoder2.getCount() - pul_prev_dir;
  pul_prev_dir = encoder2.getCount();
  return enc_dir_pul;
}

void setup()
{
  Serial.begin(115200);

  pinMode(in_esq1, OUTPUT);
  pinMode(in_esq2, OUTPUT);
  pinMode(in_dir1, OUTPUT);
  pinMode(in_dir2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(s_lat_esq, INPUT);
  pinMode(s_lat_dir, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(PROPELLER_PIN, OUTPUT);
  pinMode(boot, INPUT);

  led_stip.begin();
  SerialBT.begin("Semreh"); //Bluetooth device name

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_A, enc_eq_B);
  encoder2.attachFullQuad(enc_dir_B, enc_dir_A);

  digitalWrite(stby, HIGH);

  encoder.clearCount();
  encoder2.clearCount();

  sArray.setTypeMCP3008();
  sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
  sArray.setSamplesPerSensor(5); // VERIFICARRR

  for (int i = 0; i < 200; i++)
  {
    if(i < 100)
    {
      led_stip.setPixelColor(0,B);
      led_stip.show();
    }

    else
    {
      led_stip.setPixelColor(0,R);
      led_stip.show();
    }
    sArray.calibrate();
    delay(20);
  }
  led_stip.setPixelColor(0,0,0,0);
  led_stip.show();
  encoder.clearCount();
  encoder2.clearCount();

  //xTaskCreatePinnedToCore(ler_sens_lat_esq,"Sensor lat esq",4000,NULL,1,NULL,0);
  //xTaskCreatePinnedToCore(ler_sens_lat_dir,"Sensor lat dir",4000,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(ler_laterais,"Sensores Laterais",4000,NULL,1,NULL,0);
}

void loop()
{  
  bluetoothRead();

  callRobotTask(lastReceivedChar);

  // digitalWrite(in_dir1,LOW);
  // digitalWrite(in_dir2,HIGH);
  // analogWrite(PWM_RIGHT,200);

  // digitalWrite(in_esq1,LOW);
  // digitalWrite(in_esq2,HIGH);
  // analogWrite(PWM_LEFT,190);

  // calcula_PID_rot(KpParamRot,KdParamRot,KiParamRot);
  // controle_motores(140);
}