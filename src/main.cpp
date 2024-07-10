#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

#include <cmath>
#include <vector>
#include <iostream>
#include "FS.h"
#include <LittleFS.h>
#include <string>


using namespace std;

class Map_Data{
public:
  float leftEncoderCount;
  float rightEncoderCount;
  float meanEncoderCount;
  float leftEncoderDelta;
  float rightEncoderDelta;
  float meanEncoderDelta;
  float curveSpeed;
  float lineSpeed;
  bool curve;
  float accelerationSpace;
  float desaccelerationSpace;
  float accelerationCount;
  float desaccelerationCount;

  Map_Data(float leftEncoderCount = 0, float rightEncoderCount = 0, float meanEncoderCount = 0,
              float leftEncoderDelta = 0, float rightEncoderDelta = 0, float curveSpeed = 0,
              float lineSpeed = 0, bool state = false, float accelerationSpace = 0,
              float desaccelerationSpace = 0, float accelerationCount = 0, float desaccelerationCount = 0)
      : leftEncoderCount(leftEncoderCount), rightEncoderCount(rightEncoderCount), meanEncoderCount((leftEncoderCount + rightEncoderCount)/2),
      leftEncoderDelta(leftEncoderDelta), rightEncoderDelta(rightEncoderDelta), meanEncoderDelta((leftEncoderDelta + rightEncoderDelta)/2), curveSpeed(curveSpeed),
      lineSpeed(lineSpeed), curve(curve), accelerationSpace(accelerationSpace), desaccelerationSpace(desaccelerationSpace), accelerationCount(accelerationCount), desaccelerationCount(desaccelerationCount){}
};

std::vector<Map_Data> mapDataList;

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

bool FirstTimeOnSwitchCase = true;

long int calculate_rpm_esq();
long int calculate_rpm_dir();

void writeFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    SerialBT.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    SerialBT.println("- file written");
  } else {
    SerialBT.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    SerialBT.println("- failed to open file for appending");
    return;
  }
  if(file.print(message)){
    SerialBT.println("- message appended");
  } else {
    SerialBT.println("- append failed");
  }
  file.close();
}

void readFile(fs::FS &fs, const char * path){
  File file = fs.open(path);
  if(!file || file.isDirectory()){
    SerialBT.println("- failed to open file for reading");
    return;
  }

  static int lineIndex = 0;
  String dataString;
  while (file.available()) {
    dataString = file.readStringUntil('\n');
    if (dataString.length() > 0) {
      // Separa as informações em um vetor de strings
      int commaIndex = dataString.indexOf(',');
      String meanEncoderCount = dataString.substring(0, commaIndex);

      int secondCommaIndex = dataString.indexOf(',', commaIndex + 1);
      String curve = dataString.substring(commaIndex + 1, secondCommaIndex);

      int thirdCommaIndex = dataString.indexOf(',', secondCommaIndex + 1);
      String curveSpeed = dataString.substring(secondCommaIndex + 1, thirdCommaIndex);

      int fourthCommaIndex = dataString.indexOf(',', thirdCommaIndex + 1);
      String accelerationCount = dataString.substring(thirdCommaIndex + 1, fourthCommaIndex);

      String desaccelerationCount = dataString.substring(fourthCommaIndex + 1);
      
      mapDataList[lineIndex].meanEncoderCount = atof(meanEncoderCount.c_str());
      mapDataList[lineIndex].curveSpeed = atof(curveSpeed.c_str());
      mapDataList[lineIndex].accelerationCount = atof(accelerationCount.c_str());
      mapDataList[lineIndex].desaccelerationCount = atof(desaccelerationCount.c_str());
      mapDataList[lineIndex].curve = atoi(curve.c_str());

      SerialBT.println(mapDataList[lineIndex].meanEncoderCount);
      SerialBT.println(mapDataList[lineIndex].curveSpeed);
      SerialBT.println(mapDataList[lineIndex].accelerationCount);
      SerialBT.println(mapDataList[lineIndex].desaccelerationCount);
      SerialBT.println(mapDataList[lineIndex].curve);

      lineIndex++;
    }
  }
  SerialBT.println("Terminei de passar os dados pra RAM");
  file.close();
}

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

// void calcula_PID_rot(float KpParamRot , float KdParamRot , float KiParamrot){
//   erro_f_rot = (calculate_rpm_esq() - calculate_rpm_dir());
//   P_rot = erro_f_rot;
//   D_rot = erro_f_rot - erro_anterior_rot;
//   I_rot += erro_f_rot;
//   PIDrot =(KpParamRot*P_rot)+(KdParamRot*D_rot)+(KiParamRot*I_rot);
//   erro_anterior_rot = erro_f_rot;
// }

// float curva_acel(float pwm_goal)
// {

//   if (pwm_goal > last_pwm)
//   {
//     last_pwm += 10;
//     run_pwm = last_pwm;
//   }
//   else
//   {
//     run_pwm = pwm_goal;
//   }
//   return run_pwm;
// }

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

// void controle_motores(float motorPWM)
// {
//   velesq = curva_acel(motorPWM) + PID;
//   veldir = curva_acel(motorPWM) - PID;

//   if(veldir >= 0)
//   {
//     if(veldir > MAX_PWM) veldir = MAX_PWM;
//     analogWrite(in_dir1,LOW);
//     analogWrite(in_dir2,veldir);
//   }
//   else
//   {
//     veldir = (-1) * veldir;
//     analogWrite(in_dir1,veldir);
//     analogWrite(in_dir2,LOW);
//   }
//   //analogWrite(PWM_RIGHT,veldir);

//   if(velesq >= 0)
//   {
//     if (velesq > MAX_PWM) velesq = MAX_PWM;
//     analogWrite(in_esq1,LOW);
//     analogWrite(in_esq2,velesq);
//   }
//   else
//   {
//     velesq = (-1) * velesq;
//     analogWrite(in_esq1,velesq);
//     analogWrite(in_esq2,LOW);
//   }
//   //analogWrite(PWM_LEFT,velesq);
// }

// void controle_motores_rot(float motorPWM)
// {
//   velesqrot = motorPWM + PIDrot;
//   veldirrot = motorPWM - PIDrot;

//   if(veldirrot >= 0)
//   {
//     if(veldirrot > MAX_PWM) veldirrot = MAX_PWM;
//     digitalWrite(in_dir1,LOW);
//     digitalWrite(in_dir2,HIGH);
//   }
//   else
//   {
//     veldirrot = (-1) * veldirrot;
//     digitalWrite(in_dir1,HIGH);
//     digitalWrite(in_dir2,LOW);
//   }
//   analogWrite(PWM_RIGHT,veldirrot);

//   if(velesqrot >= 0)
//   {
//     if (velesqrot > MAX_PWM) velesqrot = MAX_PWM;
//     digitalWrite(in_esq1,LOW);
//     digitalWrite(in_esq2,HIGH);
//   }
//   else
//   {
//     velesqrot = (-1) * velesqrot;
//     digitalWrite(in_esq1,HIGH);
//     digitalWrite(in_esq2,LOW);
//   }
//   analogWrite(PWM_LEFT,velesqrot);
// }

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

// struct MotorControlData {
//     float encoderValueBegin;
//     float encoderValueEnd;
//     int motorPWM;
// };

// std::vector<MotorControlData> trackMap = {
//   //{encoderValueBegin, encoderValueEnd, motorPWM},
//   {0,	3000,	200},
//   {4000,	6000,	120},
//   {7300, 16000, 250},
//   {18800, 22000, 170},
//   //{25000, 30000, 60},
//   //{72000, 99999, 0}
// };

// void motorControlWithMap(int encVal)
// {
//   bool not_mapped = true;
//   for (const auto& MotorControlData : trackMap)
//   {
//     float firstEncoderValue = MotorControlData.encoderValueBegin;
//     float secondEncoderValue = MotorControlData.encoderValueEnd;
//     int pwmDeliveredToMotors = MotorControlData.motorPWM;

//     if (encVal >= firstEncoderValue && encVal <= secondEncoderValue)
//     {
//       if(pwmDeliveredToMotors>=220)
//       {
//         calcula_PID(KpR, KdR);
//         controle_motores(pwmDeliveredToMotors);
//       }

//       else
//       { 
//         calcula_PID(Kp, Kd);
//         controle_motores(pwmDeliveredToMotors);
//       }
//       not_mapped = false;
//       //return;
//     }
//   }
//   if(not_mapped){
//     calcula_PID(Kp, Kd);
//     controle_motores(100);
//   }
// }

void controle_com_mapeamento(int encVal)
{
  if(encVal<357) {calcula_PID(Kp,Kd); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 1.5); controle_motores_translacional();} //começo de pista lateral direito até primeira marcaçao
  else if(encVal >= 20100 && encVal <= 20437) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 4); controle_motores_translacional();}
  else if(encVal >= 23600 && encVal <= 26000) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 5); controle_motores_translacional();}
  else if(encVal >= 26001 && encVal <= 26200) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 4); controle_motores_translacional();}
  else if(encVal >= 26201 && encVal <= 26400) {calcula_PID(KpR,KdR); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 3); controle_motores_translacional();}
  
  else{calcula_PID(Kp,Kd); calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 2); controle_motores_translacional();}
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
        SerialBT.println("INICIO/FIM");
        encoder2.clearCount();
        encoder.clearCount();
        firstTimeRight = true;
        readingWhiteRight = true;
        //mapDataList.push_back(Map_Data(encoder.getCount(), encoder2.getCount(), (encoder.getCount()+encoder2.getCount())/2));
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
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(SAMPLING_TIME));  // Pausa de 10ms entre as verificações
    leftDistanceTravelled = encoder.getCount() - leftEncoderPulse;
    rightDistanceTravelled = encoder2.getCount() - rightEncoderPulse;
    robotSpeed = ((leftDistanceTravelled + rightDistanceTravelled)/2)* MM_PER_COUNT / SAMPLING_TIME; //m/s
  }
}

void tratamento()
{
  if(!mapDataList.empty()) //verifica se a lista não está vazia
  {
    for(size_t i = 0; i < mapDataList.size(); i++) //percorre a lista
    {
      Map_Data& currentData = mapDataList[i]; //pega o endereço do objeto do tipo Map_Data atual a lista
      if(i == 0) //caso seja o primeiro item da lista, a variação de espaço vai ser igual ao quanto o robô andou até agora
      {
        currentData.leftEncoderDelta = currentData.leftEncoderCount;
        currentData.rightEncoderDelta = currentData.rightEncoderCount;
      }
      else //se não for o primeiro da lista, a variação de espaço é igual ao quanto ele andou até agora menos o quanto ele andou até a marcação anterior
      {
        Map_Data& previousData = mapDataList[i-1];
        currentData.leftEncoderDelta = currentData.leftEncoderCount - previousData.leftEncoderCount;
        currentData.rightEncoderDelta = currentData.rightEncoderCount - previousData.rightEncoderCount;
      }
      currentData.meanEncoderCount = (currentData.leftEncoderCount + currentData.rightEncoderCount)/2;
      
      //transforma o delta pulsos para delta em metros
      float leftEncoderDeltaMeter = (MM_PER_COUNT * currentData.leftEncoderDelta)/1000;
      float rightEncoderDeltaMeter = (MM_PER_COUNT * currentData.rightEncoderDelta)/1000;
      float meanEncoderDeltaMeter = (leftEncoderDeltaMeter + rightEncoderDeltaMeter)/2;

      currentData.meanEncoderDelta = meanEncoderDeltaMeter;

      if(leftEncoderDeltaMeter == rightEncoderDeltaMeter) //verificação para que não haja divisão por zero no cálculo de curveRadius
      {
        currentData.curve = false;
      }
      else
      {
        //calcula o raio da curva
        float curveRadius = abs((DISTANCEWHEELTOCENTER/2) * ((leftEncoderDeltaMeter+rightEncoderDeltaMeter)/(leftEncoderDeltaMeter-rightEncoderDeltaMeter)));

        if(curveRadius <= 0.5) //se o raio da curva for menor ou igual do que 50cm(0.5m), então é uma curva 
        {
          currentData.curve = true;
          currentData.curveSpeed = pow((curveRadius*FRICTION/MASS)*(MASS*GRAVITY+BRUSHLESSFORCE),0.5); //calcula a velocidade para fazer a curva
        }
        else //se for maior é uma reta
        {
          currentData.curve = false;
        }
      }
    }

    SerialBT.print("Media Encoder Absoluto ");
    SerialBT.print("\t");

    SerialBT.print("Media Encoder Delta ");
    SerialBT.print("\t");

    SerialBT.print("Vcurva ");
    SerialBT.print("\t");

    SerialBT.print("Curva? ");
    SerialBT.println("\t");

    for(short i = 0; i < mapDataList.size(); i++)
    {
      SerialBT.print(mapDataList[i].meanEncoderCount);SerialBT.print("\t");
      SerialBT.print(mapDataList[i].meanEncoderDelta);SerialBT.print("\t");
      SerialBT.print(mapDataList[i].curveSpeed);SerialBT.print("\t");
      SerialBT.print(mapDataList[i].curve);SerialBT.println("\t");
    }
  }
  else
  {
    SerialBT.println("A lista está vazia");
  }

  float accelerationSpaceMeter;
  float desaccelerationSpaceMeter;
  for(int i = 0; i < mapDataList.size(); i++)
  {
    if(mapDataList[i].curve == false) //entra se for uma reta
    {
      //calcula o espaço para aceleração e desaceleração
      accelerationSpaceMeter = (pow(MAXSPEED,2) - pow(mapDataList[i-1].curveSpeed,2))/(2*acceleration);
      desaccelerationSpaceMeter = (pow(mapDataList[i+1].curveSpeed,2) - pow(MAXSPEED,2))/(2*acceleration);

      //transforma metros em pulsos de encoder
      mapDataList[i].accelerationSpace = (accelerationSpaceMeter/MM_PER_COUNT)/1000;
      mapDataList[i].desaccelerationSpace = (desaccelerationSpaceMeter/MM_PER_COUNT)/1000;
    }
    if(i==0)
    {
      if((accelerationSpaceMeter + desaccelerationSpaceMeter) < mapDataList[i].meanEncoderDelta) //se a soma dos espaços for menor do que o tamanho da reta
      {
        mapDataList[i].accelerationCount = mapDataList[i].accelerationSpace;
        mapDataList[i].desaccelerationCount = mapDataList[i].meanEncoderCount - mapDataList[i].desaccelerationSpace;
      }
      else
      {
        mapDataList[i].accelerationCount = (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)/1000)/2);
      }
    }
    else
    {
      if((accelerationSpaceMeter + desaccelerationSpaceMeter) < mapDataList[i].meanEncoderDelta) //se a soma dos espaços for menor do que o tamanho da reta
      {
        mapDataList[i].accelerationCount = mapDataList[i-1].meanEncoderCount + mapDataList[i].accelerationSpace;
        mapDataList[i].desaccelerationCount = mapDataList[i].meanEncoderCount - mapDataList[i].desaccelerationSpace;
      }
      else
      {
        mapDataList[i].accelerationCount = (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)/1000)/2)+mapDataList[i-1].meanEncoderCount;
      }
    }
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
    calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 0.5);
    controle_motores_translacional();
  break;

  case '2': //Run with track map
    static short i = 0;
    if(i < mapDataList.size())
    {
      float quantoAndouAteAgora = (encoder.getCount() + encoder2.getCount())/2; //calcula o quanto já andou até o momento
      if(quantoAndouAteAgora <= mapDataList[i].meanEncoderCount) //se o valor de encoder da marcação for maior do que o quanto andou até o momento
      {
        if(i != mapDataList.size()-1)
        {
          if(mapDataList[i].curve == false) //entra no if se for uma reta
          {
            if(quantoAndouAteAgora < mapDataList[i].accelerationCount)
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 2.5);
              controle_motores_translacional();
            }
            else if(quantoAndouAteAgora >= mapDataList[i].desaccelerationCount)
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 1);
              controle_motores_translacional();
            }
            else
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 2.5);
              controle_motores_translacional();
            }
          }
          else //caso não seja uma reta, ou seja curva
          {
            ler_sensores();
            calcula_PID(Kp,Kd);
            calcula_PID_translacional(KpTrans, KdTrans, KiTrans, mapDataList[i].curveSpeed);
            controle_motores_translacional();
          }
        }
        else //é o ultimo valor da lista
        {
          //controla o robô na pista com a velocidade para fazer a curva
          ler_sensores();
          calcula_PID(Kp,Kd);
          calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 2);
          controle_motores_translacional();
        }
      }
      else //se ele já passou da marcação lateral, então incrementa o i, indo para o próximo intervalo
      {
        i++;
      }
    }
    else //se acabou a lista para os motores
    {
      analogWrite(in_dir1,255);
      analogWrite(in_dir2,255);

      analogWrite(in_esq1,255);
      analogWrite(in_esq2,255);
      Brushless.write(0);
    }
  break;

  case '3': //Abort
    analogWrite(in_dir1,255);
    analogWrite(in_dir2,255);

    analogWrite(in_esq1,255);
    analogWrite(in_esq2,255);

    analogWrite(PROPELLER_PIN, 0);
    Brushless.write(0);
  break;

  case '4': //No zig
    ler_sensores();
    controle_com_mapeamento((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '5': //adiciona marcação
    // SerialBT.print(';'); 
    // SerialBT.print(encoder.getCount()); 
    // SerialBT.print(';'); 
    // SerialBT.print(encoder2.getCount());
    status = '1';
    lastReceivedChar = '1';

    mapDataList.push_back(Map_Data(encoder.getCount(), encoder2.getCount(), (encoder.getCount()+encoder2.getCount())/2));
  break;

  case '6': //Put the Map_Data from flash memory to RAM
  // static bool firstTimeOnFlashToRAM = true;
  // if(firstTimeOnFlashToRAM == true)
  // {
    readFile(LittleFS, "/Map_Data.txt");
  //   firstTimeOnFlashToRAM = false;
  // }
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

  case '8': //Tratamento
    static bool firstTimeProcess = true;
    tratamento();
    encoder.clearCount();
    encoder2.clearCount();
    if(firstTimeProcess == true)
    {
      for(int i = 0; i < mapDataList.size(); i++)
      {
        if (i == 0)
        {
          string meanEncoderCount = to_string(mapDataList[i].meanEncoderCount);
          string curve = to_string(mapDataList[i].curve);
          string curveSpeed = to_string(mapDataList[i].curveSpeed);
          string accelerationCount = to_string(mapDataList[i].accelerationCount);
          string desaccelerationCount = to_string(mapDataList[i].desaccelerationCount);

          string dataString = meanEncoderCount + "," + curve + "," + curveSpeed + "," + accelerationCount + "," + desaccelerationCount + "\n";
          writeFile(LittleFS, "/Map_Data.txt", dataString.c_str());
        }
        else
        {
          string meanEncoderCount = to_string(mapDataList[i].meanEncoderCount);
          string curve = to_string(mapDataList[i].curve);
          string curveSpeed = to_string(mapDataList[i].curveSpeed);
          string accelerationCount = to_string(mapDataList[i].accelerationCount);
          string desaccelerationCount = to_string(mapDataList[i].desaccelerationCount);

          string dataString = meanEncoderCount + "," + curve + "," + curveSpeed + "," + accelerationCount + "," + desaccelerationCount + "\n";
          appendFile(LittleFS, "/Map_Data.txt", dataString.c_str());
        }
      }
      firstTimeProcess = false;
    }    
    status = '3';
    lastReceivedChar = '3';
  break;

  default:
    analogWrite(PWM_LEFT,0);
    analogWrite(PWM_RIGHT,0);
    //analogWrite(PROPELLER_PIN, 0);
  break;
  }
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

  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)){
        SerialBT.println("LittleFS Mount Failed");
        return;
  }

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
  //xTaskCreatePinnedToCore(ler_laterais,"Sensores Laterais",4000,NULL,1,NULL,0);
}

void loop()
{  
  bluetoothRead();

  callRobotTask(lastReceivedChar);
}
