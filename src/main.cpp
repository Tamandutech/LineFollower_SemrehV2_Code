#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

Servo Brushless;
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
    last_pwm += 10;
    run_pwm = last_pwm;
  }
  else
  {
    run_pwm = pwm_goal;
  }
  return run_pwm;
}

float translacionalErrorBuffer[9]; // buffer to store the last 5 values of translationalError
int translacionalErrorIndex = 0; // index to keep track of the current position in the buffer
void calcula_PID_translacional(float KpParam_Translacional, float KdParam_Translacional,float KiParam_Translacional, float desiredSpeed)
{
  translacionalError = desiredSpeed - robotSpeed;
  P_Translacional = translacionalError;
  D_Translacional = translacionalError - lastTranslacionalError;

  // update the buffer and calculate the sum of the last 5 values
  translacionalErrorBuffer[translacionalErrorIndex] = translacionalError;
  translacionalErrorIndex = (translacionalErrorIndex + 1) % 9;
  float sum = 0;
  for (int i = 0; i < 9; i++) {
    sum += translacionalErrorBuffer[i];
  }
  I_Translacional = sum;

  PIDTranslacional = (KpParam_Translacional * P_Translacional) + (KdParam_Translacional * D_Translacional) + (KiParam_Translacional * I_Translacional);
  lastTranslacionalError = translacionalError;
}

void controle_motores(float motorPWM)
{
  velesq = curva_acel(motorPWM) + PID;
  veldir = curva_acel(motorPWM) - PID;

  if(veldir >= 0)
  {
    if(veldir > MAX_PWM) veldir = MAX_PWM;
    analogWrite(in_dir1,LOW);
    analogWrite(in_dir2,veldir);
  }
  else
  {
    veldir = (-1) * veldir;
    analogWrite(in_dir1,veldir);
    analogWrite(in_dir2,LOW);
  }
  //analogWrite(PWM_RIGHT,veldir);

  if(velesq >= 0)
  {
    if (velesq > MAX_PWM) velesq = MAX_PWM;
    analogWrite(in_esq1,LOW);
    analogWrite(in_esq2,velesq);
  }
  else
  {
    velesq = (-1) * velesq;
    analogWrite(in_esq1,velesq);
    analogWrite(in_esq2,LOW);
  }
  //analogWrite(PWM_LEFT,velesq);
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

void controle_motores_translacional()
{
  velesq = PIDTranslacional + PID;
  veldir = PIDTranslacional - PID;

  if(veldir >= 0)
  {
    if(veldir > MAX_PWM) veldir = MAX_PWM;
    analogWrite(in_dir1,LOW);
    analogWrite(in_dir2,veldir);
  }
  else
  {
    veldir = (-1) * veldir;
    analogWrite(in_dir1,veldir);
    analogWrite(in_dir2,LOW);
  }

  if(velesq >= 0)
  {
    if (velesq > MAX_PWM) velesq = MAX_PWM;
    analogWrite(in_esq1,LOW);
    analogWrite(in_esq2,velesq);
  }
  else
  {
    velesq = (-1) * velesq;
    analogWrite(in_esq1,velesq);
    analogWrite(in_esq2,LOW);
  }
}

struct MotorControlData {
    float encoderValueBegin;
    float encoderValueEnd;
    int motorPWM;
};

std::vector<MotorControlData> trackMap = {
  //{encoderValueBegin, encoderValueEnd, motorPWM},
  {0,	3000,	200},
  {4000,	6000,	120},
  {7300, 16000, 250},
  {18800, 22000, 170},
  //{25000, 30000, 60},
  //{72000, 99999, 0}
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
  if(encVal<357) {calcula_PID(Kp,Kd); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 1.5); controle_motores_translacional();} //começo de pista lateral direito até primeira marcaçao
  else if(encVal >= 20100 && encVal <= 20437) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 4); controle_motores_translacional();}
  else if(encVal >= 23600 && encVal <= 26000) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 5); controle_motores_translacional();}
  else if(encVal >= 26001 && encVal <= 26200) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 4); controle_motores_translacional();}
  else if(encVal >= 26201 && encVal <= 26400) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 3); controle_motores_translacional();}
  
  else{calcula_PID(Kp,Kd); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 2); controle_motores_translacional();}
}

void controle_com_mapeamento2(int encVal)
{
  if(encVal<2750) {calcula_PID(KpR,KdR); controle_motores(230);Brushless.write(55);} //começo de pista lateral direito até primeira marcaçao
  else if(encVal >= 2750 && encVal <= 4000) {calcula_PID(Kp,Kd); controle_motores(125);Brushless.write(100);} // primeira curva
  else if(encVal >= 4000 && encVal <= 6500) {calcula_PID(Kp,Kd); controle_motores(125);Brushless.write(100);} // primeira curva
  else if(encVal >= 7370 && encVal <= 16000) {calcula_PID(KpR,KdR); controle_motores(250); Brushless.write(55);}
  else if(encVal >= 18400 && encVal <= 22000) {calcula_PID(Kp,Kd); controle_motores(170); Brushless.write(80);}
  else if(encVal >= 22000 && encVal <= 32000) {calcula_PID(Kp,Kd); controle_motores(160); Brushless.write(145);}
  // else if(encVal >= 27800 && encVal <= 28500) {calcula_PID(Kp,Kd); controle_motores(100);}
  //else if(encVal >= 26000 && encVal <= 30000) {calcula_PID(Kp,Kd); controle_motores(70); Brushless.write(30);}
  // else if(encVal >= 30900 && encVal <= 32700) {calcula_PID(Kp,Kd); controle_motores(130);Brushless.write(90);}
  // else if(encVal >= 34500 && encVal <= 36000) {calcula_PID(Kp,Kd); controle_motores(95);}
  else if(encVal >= 32000 && encVal <= 35700) {calcula_PID(Kp,Kd); controle_motores(160);Brushless.write(110);}
  else if(encVal >= 36750 && encVal <= 40750) {calcula_PID(KpR,KdR); controle_motores(250);Brushless.write(55);} //reta 2
  else if(encVal >= 41000 && encVal <= 43000) {calcula_PID(Kp,Kd); controle_motores(90);Brushless.write(80);} //curva pós reta 2
  // //else if(encVal >= 43600 && encVal <= 44000) {calcula_PID(KpR,KdR); controle_motores(90);}
  else if(encVal >= 43000 && encVal <= 45250) {calcula_PID(Kp,Kd); controle_motores(110); Brushless.write(90);}


  else if(encVal >= 45251 && encVal <= 46150) {calcula_PID(Kp,Kd); controle_motores(210);} //Primeira reta cruzada

  else if(encVal >= 47970 && encVal <= 48700) {calcula_PID(Kp,Kd); controle_motores(210);} //Segunda reta cruzada

  else if(encVal >= 50350 && encVal <= 51150) {calcula_PID(Kp,Kd); controle_motores(210);} //Terceira reta cruzada

  else if(encVal >= 53070 && encVal <= 53870) {calcula_PID(Kp,Kd); controle_motores(210);} //Quarta reta cruzada

  else if(encVal >= 55400 && encVal <= 56120) {calcula_PID(Kp,Kd); controle_motores(210);} //Quinta reta cruzada

  else if(encVal >= 58000 && encVal <= 59100) {calcula_PID(Kp,Kd); controle_motores(100); Brushless.write(90);} //Prep Reta 3

  else if(encVal >= 59100 && encVal <= 61000) {calcula_PID(Kp,Kd); controle_motores(210);Brushless.write(70);} //Reta 3

  else if(encVal >= 61000 && encVal <= 62500) {calcula_PID(Kp,Kd); controle_motores(100); Brushless.write(110);} //Prep Reta 4

  else if(encVal >= 62500 && encVal <= 64500) {calcula_PID(Kp,Kd); controle_motores(210); Brushless.write(70);} //Reta 4

  else if(encVal >= 64500 && encVal <= 66000) {calcula_PID(Kp,Kd); controle_motores(100); Brushless.write(100);} //Prep Reta 5

  else if(encVal >= 66500 && encVal <= 67700) {calcula_PID(Kp,Kd); controle_motores(210); Brushless.write(70); led_stip.setPixelColor(0,R); led_stip.show();} //Reta 5

  else if(encVal >= 67800 && encVal <= 70300) {calcula_PID(Kp,Kd); controle_motores(120); Brushless.write(130); led_stip.setPixelColor(0,G); led_stip.show();}

  else if(encVal >= 70300 && encVal <= 74500) {calcula_PID(Kp,Kd); controle_motores(160); Brushless.write(130); led_stip.setPixelColor(0,R); led_stip.show();}

  else if (encVal >= 74500) {calcula_PID(Kp,Kd); controle_motores(0); Brushless.write(0);}
  else{calcula_PID(Kp,Kd); controle_motores(115); Brushless.write(80);}
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
        vTaskDelay(pdMS_TO_TICKS(5));  // Manter o buzzer ligado por 100ms
        led_stip.setPixelColor(1, 0, 0, 0);
        led_stip.show();
        readingWhiteLeft = true;
      }

      else if(medLateralEsq > 3000 && medLateralDir < 2900 && readingWhiteRight == false && firstTimeRight == false) //lê apenas marcação direita
      {
        tempo = millis();
        SerialBT.println("INICIO/FIM");
        encoder2.clearCount();
        encoder.clearCount();
        firstTimeRight = true;
        readingWhiteRight = true;
      }
    }

    else
    {
      readingWhiteLeft = false;
      readingWhiteRight = false;
    }
    medLateralDir = 0;
    medLateralEsq = 0;

    countLateral = countLateral % MED_TAMANHO;
    vTaskDelay(pdMS_TO_TICKS(8));  // Pausa de 5ms entre as verificações
  }
}

void calculateRobotSpeed(void *parameter) //m/s
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    leftEncoderPulse = encoder.getCount();
    rightEncoderPulse = encoder2.getCount();
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(10));  // Pausa de 10ms entre as verificações
    leftDistanceTravelled = encoder.getCount() - leftEncoderPulse;
    rightDistanceTravelled = encoder2.getCount() - rightEncoderPulse;
    robotSpeed = ((leftDistanceTravelled + rightDistanceTravelled)/2)* MM_PER_COUNT / SAMPLING_TIME; //m/s
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
    calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 0.75);
    controle_motores_translacional();
  break;

  case '2': //Run with track map
    ler_sensores();
    motorControlWithMap((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '3': //Abort
    pinMode(in_dir1, OUTPUT);
    pinMode(in_dir2, OUTPUT);
    pinMode(in_esq1, OUTPUT);
    pinMode(in_esq2, OUTPUT);

    digitalWrite(in_dir1,HIGH);
    digitalWrite(in_dir2,HIGH);

    digitalWrite(in_esq1,HIGH);
    digitalWrite(in_esq2,HIGH);

    analogWrite(PROPELLER_PIN, 0);
    Brushless.write(0);
  break;

  case '4': //No zig
    ler_sensores();
    controle_com_mapeamento((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '5': //No zig 2 (-240)
    ler_sensores();
    controle_com_mapeamento2((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '6': //Propeller
    static bool firstTimeOnPropeller = true;
    if(firstTimeOnPropeller == true)
    {
       for(int i=1; i<PROPELLER_PWM; i++)
      {
        analogWrite(PROPELLER_PIN,i);
        delay(10);
      }
      firstTimeOnPropeller = false;
    }
    analogWrite(PROPELLER_PIN,PROPELLER_PWM);
  break;

  case '7':
    static bool firstTimeOnBrushless = true;
    if(firstTimeOnBrushless == true)
    {
      for(int i=1; i<BRUSHLESSSPEED; i++)
      {
        Brushless.write(i);
        delay(25);
      }
      firstTimeOnBrushless = false;
    }
    Brushless.write(BRUSHLESSSPEED);
  break;

  default:
    analogWrite(PWM_LEFT,0);
    analogWrite(PWM_RIGHT,0);
    analogWrite(PROPELLER_PIN, 0);
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
  pinMode(BRUSHLESS_PIN, OUTPUT);
  pinMode(boot, INPUT);

  led_stip.begin();
  SerialBT.begin("Semreh"); //Bluetooth device name

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_A, enc_eq_B);
  encoder2.attachFullQuad(enc_dir_B, enc_dir_A);

  digitalWrite(stby, HIGH);

  encoder.clearCount();
  encoder2.clearCount();

  Brushless.attach(BRUSHLESS_PIN, 1000, 2000);

  Brushless.write(180);
  delay(5000);
  Brushless.write(0);


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

  xTaskCreatePinnedToCore(calculateRobotSpeed,"Velocidade",10000,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(ler_laterais,"Sensores Laterais",4000,NULL,1,NULL,0);
}

void loop()
{  
  bluetoothRead();

  callRobotTask(lastReceivedChar);
}
