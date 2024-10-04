#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <cmath>
#include <vector>
#include <iostream>
// #include "FS.h"
// #include <LittleFS.h>
#include <string>
#include <fstream>
#include "SPIFFS.h"


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
  int curve;
  float accelerationSpace;
  float desaccelerationSpace;
  float accelerationCount;
  float desaccelerationCount;

  Map_Data(float leftEncoderCount = 0, float rightEncoderCount = 0, float meanEncoderCount = 0,
              float leftEncoderDelta = 0, float rightEncoderDelta = 0, float meanEncoderDelta = 0, float curveSpeed = 0,
              float lineSpeed = 0, int curve = 0, float accelerationSpace = 0,
              float desaccelerationSpace = 0, float accelerationCount = 0, float desaccelerationCount = 0)
      : leftEncoderCount(leftEncoderCount), rightEncoderCount(rightEncoderCount), meanEncoderCount(meanEncoderCount),
      leftEncoderDelta(leftEncoderDelta), rightEncoderDelta(rightEncoderDelta), meanEncoderDelta((leftEncoderDelta + rightEncoderDelta)/2), curveSpeed(curveSpeed),
      lineSpeed(lineSpeed), curve(curve), accelerationSpace(accelerationSpace), desaccelerationSpace(desaccelerationSpace), accelerationCount(accelerationCount), desaccelerationCount(desaccelerationCount){}
};

std::vector<Map_Data> mapDataListManual;
std::vector<Map_Data> mapDataListSensor;
std::vector<Map_Data> mapDataList;

Servo Brushless;
ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;
BluetoothSerial SerialBT;
Adafruit_NeoPixel led_stip(LED_COUNT, led, NEO_GRB + NEO_KHZ800); // Declare our NeoPixel strip object
Adafruit_MPU6050 mpu6050;
TwoWire I2CMPU6050 = TwoWire(0);

const uint32_t R = led_stip.Color(255,0,0);
const uint32_t G = led_stip.Color(0,255,0);
const uint32_t B = led_stip.Color(0,0,255);
const uint32_t W = led_stip.Color(255,255,255);

bool FirstTimeOnSwitchCase = true;

long int calculate_rpm_esq();
long int calculate_rpm_dir();

void writeFile(const char * path, const char * message){
  File file = SPIFFS.open(path, FILE_WRITE);
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

void appendFile(const char * path, const char * message){
  File file = SPIFFS.open(path, FILE_APPEND);
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

void deleteFile(fs::FS &fs, const char * path){
    if(fs.remove(path)){
        SerialBT.println("- file deleted");
    } else {
        SerialBT.println("- delete failed");
    }
}

void readFile(const char * path){
  File file = SPIFFS.open(path);
  if(!file){
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
      String stringCurve = dataString.substring(commaIndex + 1, secondCommaIndex);

      int thirdCommaIndex = dataString.indexOf(',', secondCommaIndex + 1);
      String curveSpeed = dataString.substring(secondCommaIndex + 1, thirdCommaIndex);

      int fourthCommaIndex = dataString.indexOf(',', thirdCommaIndex + 1);
      String accelerationCount = dataString.substring(thirdCommaIndex + 1, fourthCommaIndex);

      String desaccelerationCount = dataString.substring(fourthCommaIndex + 1);
      
      float lmeanEncoderCount = atof(meanEncoderCount.c_str());
      float lcurveSpeed = atof(curveSpeed.c_str());
      float laccelerationCount = atof(accelerationCount.c_str());
      float ldesaccelerationCount = atof(desaccelerationCount.c_str());
      int curve = atoi(stringCurve.c_str());

      SerialBT.print(lmeanEncoderCount);
      SerialBT.print(",");
      SerialBT.print(curve);
      SerialBT.print(",");
      SerialBT.print(lcurveSpeed);
      SerialBT.print(",");
      SerialBT.print(laccelerationCount);
      SerialBT.print(",");
      SerialBT.println(ldesaccelerationCount);
      
      
      mapDataList.push_back(Map_Data(0.0f, 0.0f, lmeanEncoderCount, 0.0f,0.0f,0.0f,lcurveSpeed,0.0f,curve,0.0f,0.0f,laccelerationCount,ldesaccelerationCount));
    }
  }
  SerialBT.println("Terminei de passar os dados pra RAM");
  // for(short i = 0; i<mapDataList.size();i++)
  // {
  //   SerialBT.println(mapDataList[i].meanEncoderCount);
  // }
  // SerialBT.println(path);
  // String filePath = ".";
  //File outputFile = SD.open(filePath, FILE_WRITE);

  
  file.close();
}

// void escreverArquivo(const char * filename){
//   String filePath = "TestMap.txt" ;
//   File inputFile = SD.open(filePath,FILE_READ );
//   File file = LittleFS.open(filename, "w");

//   while(inputFile.available()){
//     file.write(inputFile.read());
//     SerialBT.write(inputFile.read());
//   }
  
//   file.close();
//   inputFile.close();

// }

void escreverArquivo(fs::FS &fs,const char* outputfile) {
  // Usar std::ifstream para abrir o arquivo no Windows
  const char* filePath = "C:\\Users\\henri\\OneDrive\\Documentos\\VSCode\\LineFollower_SemrehV2_Code\\TestMap.txt"; // Coloque o caminho completo do arquivo no Windows

  File file = fs.open(outputfile);
  if(!file || file.isDirectory()){
    SerialBT.println("- failed to create OutputFIle");
    return;
  }

  std::ifstream inputFile(filePath, std::ios::binary); // Abrir arquivo em modo binário para leitura


  if (!inputFile) {
    SerialBT.println("failed to open inputFile");
    return;
  }

  // Copiar dados do arquivo de entrada para o arquivo de saída
  char buffer;
  while (inputFile.get(buffer)) {
    file.write(buffer);
    SerialBT.println(buffer);
  }

  file.close();
  inputFile.close();
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

float rotacionalErrorBuffer[9]; // buffer to store the last 5 values of translationalError
int rotacionalErrorIndex = 0; // index to keep track of the current position in the buffer
void calcula_PID_rot(float KpParamRot , float KdParamRot , float KiParamRot){
  rotacionalError = (calculate_rpm_esq() - calculate_rpm_dir());
  P_rot = rotacionalError;
  D_rot = rotacionalError - lastRotacionalError;
  
  // update the buffer and calculate the sum of the last 5 values
  // rotacionalErrorBuffer[rotacionalErrorIndex] = rotacionalError;
  // rotacionalErrorIndex = (rotacionalErrorIndex + 1) % 9;
  // float sum = 0;
  // for (int i = 0; i < 9; i++) {
  //   sum += rotacionalErrorBuffer[i];
  // }
  // I_Translacional = sum;
  PIDrot =(KpParamRot*P_rot)+(KdParamRot*D_rot);
  lastRotacionalError = rotacionalError;
}

float curva_acel(float pwm_goal)
{

  if (pwm_goal > last_pwm)
  {
    run_pwm = pwm_goal;
  }
  else if (pwm_goal < last_pwm)
  {
    last_pwm -= 0.02f;
    run_pwm = pwm_goal;
  }
  else
  {
    run_pwm = pwm_goal;
  }
  return run_pwm;
}

float translacionalErrorBuffer[9]; // buffer to store the last 5 values of translationalError
int translacionalErrorIndex = 0; // index to keep track of the current position in the buffer

// int delta_time = micros();
// int last_time = 0;
// int n_pid = 0;

void calcula_PID_translacional(float KpParam_Translacional, float KdParam_Translacional,float KiParam_Translacional, float desiredSpeed)
{
  translacionalError = curva_acel(desiredSpeed) - robotSpeed;
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

void motorControl()
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

void motorControlOnLine()
{
  velesq = PIDTranslacional  + PIDrot;
  veldir = PIDTranslacional  - PIDrot;

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
        // SerialBT.print(';'); 
        // SerialBT.print(encoder.getCount()); 
        // SerialBT.print(';'); 
        // SerialBT.print(encoder2.getCount()); 
        // SerialBT.print(';'); 
        // SerialBT.println((encoder.getCount() + encoder2.getCount())/2);
        led_stip.setPixelColor(1, 0, 0, 255);
        led_stip.show();
        vTaskDelay(pdMS_TO_TICKS(20));  // Manter o buzzer ligado por 100ms
        led_stip.setPixelColor(1, 0, 0, 0);
        led_stip.show();
        readingWhiteLeft = true;
        mapDataList.push_back(Map_Data(encoder.getCount(), encoder2.getCount(), (encoder.getCount()+encoder2.getCount())/2));

      }

      else if(medLateralEsq > 3000 && medLateralDir < 2900 && readingWhiteRight == false && firstTimeRight == false) //lê apenas marcação direita
      {
        SerialBT.println("INICIO/FIM");
        // encoder2.clearCount();
        // encoder.clearCount();
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
        currentData.curve = 0;
      }
      else
      {
        //calcula o raio da curva
        float curveRadius = abs((DISTANCEWHEELTOCENTER/2) * ((leftEncoderDeltaMeter+rightEncoderDeltaMeter)/(leftEncoderDeltaMeter-rightEncoderDeltaMeter)));

        if(curveRadius <= 0.5) //se o raio da curva for menor ou igual do que 50cm(0.5m), então é uma curva 
        {
          currentData.curve = 1;
          currentData.curveSpeed = pow((curveRadius*FRICTION)*(GRAVITY+(BRUSHLESSFORCE/MASS)),0.5); //calcula a velocidade para fazer a curva
        }
        else //se for maior é uma reta
        {
          currentData.curve = 0;
        }
      }
    }
    for(short i = 0; i < mapDataList.size(); i++)
    {
      // SerialBT.print(mapDataList[i].meanEncoderCount);SerialBT.print("\t");
      // SerialBT.print(mapDataList[i].meanEncoderDelta);SerialBT.print("\t");
      // SerialBT.print(mapDataList[i].curveSpeed);SerialBT.print("\t");
      //SerialBT.println(mapDataList[i].curve);
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
    if(mapDataList[i].curve == 0) //entra se for uma reta
    {
      if (i != mapDataList.size()-1) //entra se não for o ultimo item da lista
      {
        if(i != 0) //entra se não for o primiero item da lista
        {
          //calcula o espaço para aceleração e desaceleração
          accelerationSpaceMeter = (pow(MAXSPEED,2) - pow(mapDataList[i-1].curveSpeed,2))/(2*acceleration);
          desaccelerationSpaceMeter = -((pow(mapDataList[i+1].curveSpeed,2) - pow(MAXSPEED,2))/(2*acceleration));

          //transforma metros em pulsos de encoder
          mapDataList[i].accelerationSpace = (accelerationSpaceMeter/MM_PER_COUNT)*1000;
          mapDataList[i].desaccelerationSpace = (desaccelerationSpaceMeter/MM_PER_COUNT)*1000;
        }
        else //entra se for o primeiro item da lista
        {
          //calcula o espaço para aceleração e desaceleração
          accelerationSpaceMeter = (pow(MAXSPEED,2))/(2*acceleration);
          desaccelerationSpaceMeter = -((pow(mapDataList[i+1].curveSpeed,2) - pow(MAXSPEED,2))/(2*acceleration));

          //transforma metros em pulsos de encoder
          mapDataList[i].accelerationSpace = (accelerationSpaceMeter/MM_PER_COUNT)*1000;
          mapDataList[i].desaccelerationSpace = (desaccelerationSpaceMeter/MM_PER_COUNT)*1000;
        }
      }
      else //se for o ultimo item da lista
      {
        //calcula o espaço para aceleração e desaceleração
        accelerationSpaceMeter = (pow(MAXSPEED,2) - pow(mapDataList[i-1].curveSpeed,2))/(2*acceleration);
        desaccelerationSpaceMeter = -((pow(2,2) - pow(MAXSPEED,2))/(2*acceleration));

        //transforma metros em pulsos de encoder
        mapDataList[i].accelerationSpace = (accelerationSpaceMeter/MM_PER_COUNT)*1000.0f;
        mapDataList[i].desaccelerationSpace = (desaccelerationSpaceMeter/MM_PER_COUNT)*1000.0f;
      }    
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
        mapDataList[i].accelerationCount = (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)*1000)/2);
        mapDataList[i].desaccelerationCount = mapDataList[i].meanEncoderCount - (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)*1000)/2);
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
        mapDataList[i].accelerationCount = (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)*1000)/2) + mapDataList[i-1].meanEncoderCount;
        mapDataList[i].desaccelerationCount = mapDataList[i].meanEncoderCount - (((mapDataList[i].meanEncoderDelta/MM_PER_COUNT)*1000)/2);
      }
    }
  }
}

void hybridMapping(const std::vector<Map_Data>& manualData, const std::vector<Map_Data>& sensorData, std::vector<Map_Data>& resultData) {
  float thresholdSort = 2000;
  resultData.resize(manualData.size());
  for (std::size_t i = 0; i < manualData.size(); ++i) {
    bool found = false;
    for (std::size_t j = 0; j < sensorData.size(); ++j) {
      if (std::abs (manualData[i].meanEncoderCount - sensorData[j].meanEncoderCount) <= thresholdSort){
        resultData[i] = sensorData[j];
        found = true;
        break;
      }
    }
    if(!found){
      resultData[i] = manualData[i];
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
uint32_t battery_adc = 0;
uint8_t brushless_Speed = 160;
TaskHandle_t BrushlessSpeedHandle = NULL;
void calculateBrushlessSpeed(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(true)
  {
    for (int i = 0; i < 500; i++)
    {
        battery_adc += analogRead(battery);
    }
    battery_adc /= 500; // Média de 500 leituras
    brushless_Speed = (brushless_Speed/160)*300000/battery_adc;
    Brushless.write(brushless_Speed);
    SerialBT.println(brushless_Speed);
    SerialBT.println(battery_adc);
    vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(1000));  // Pausa de 1s entre as verificações
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
    motorControl();
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
          if(mapDataList[i].curve == 0) //entra no if se for uma reta
          {
            if(quantoAndouAteAgora < mapDataList[i].accelerationCount)
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, MAXSPEED);
              motorControl();
              led_stip.setPixelColor(0,G);
              led_stip.show();
            }
            else if(quantoAndouAteAgora >= mapDataList[i].desaccelerationCount)
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, (mapDataList[i+1].curveSpeed));
              motorControl();
              led_stip.setPixelColor(0,R);
              led_stip.show();
            }
            else
            {
              ler_sensores();
              calcula_PID(Kp,Kd);
              calcula_PID_translacional(KpTrans, KdTrans, KiTrans, MAXSPEED);
              motorControl();
              led_stip.setPixelColor(0,B);
              led_stip.show();
            }
          }
          else //caso não seja uma reta, ou seja curva
          {
            ler_sensores();
            calcula_PID(Kp,Kd);
            calcula_PID_translacional(KpTrans, KdTrans, KiTrans,(mapDataList[i].curveSpeed));
            motorControl();
            led_stip.setPixelColor(0,255,255,255);
            led_stip.show();
          }
        }
        else //é o ultimo valor da lista
        {
          //controla o robô na pista com a velocidade para fazer a curva
          ler_sensores();
          calcula_PID(Kp,Kd);
          calcula_PID_translacional(KpTrans, KdTrans, KiTrans, 3.5);
          motorControl();
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
    // static bool firstTimeOn3 = true;
    // if(firstTimeOn3 == true)
    // {
    //   pinMode(in_esq1, OUTPUT);
    //   pinMode(in_esq2, OUTPUT);
    //   pinMode(in_dir1, OUTPUT);
    //   pinMode(in_dir2, OUTPUT);

    //   analogWrite(in_dir1,255);
    //   analogWrite(in_dir2,255);

    //   analogWrite(in_esq1,255);
    //   analogWrite(in_esq2,255);

    //   firstTimeOn3 = false;
    // }

    analogWrite(in_dir1,255);
    analogWrite(in_dir2,255);

    analogWrite(in_esq1,255);
    analogWrite(in_esq2,255);

    vTaskDelete(BrushlessSpeedHandle);
    BrushlessSpeedHandle = NULL;
    Brushless.write(0);

  break;

  case '4': //Safe
  static bool firstTimeOnFlashToRAM4 = true;
  if(firstTimeOnFlashToRAM4 == true)
  {
    readFile("/SafeSabado.txt");
    firstTimeOnFlashToRAM4 = false;
  }
  break;

  case '5': //adiciona marcação
    // SerialBT.print(';'); 
    // SerialBT.print(encoder.getCount()); 
    // SerialBT.print(';'); 
    // SerialBT.print(encoder2.getCount());
    status = '1';
    lastReceivedChar = '1';
    //SerialBT.println(robotSpeed);

    // SerialBT.print(encoder.getCount());
    // SerialBT.print(" || ");
    // SerialBT.println(encoder2.getCount());

    mapDataList.push_back(Map_Data(encoder.getCount(), encoder2.getCount(), (encoder.getCount()+encoder2.getCount())/2));
  break;

  case '6': //Put the Map_Data from flash memory to RAM
  static bool firstTimeOnFlashToRAM = true;
  if(firstTimeOnFlashToRAM == true)
  {
    readFile("/Map_Data.txt");
    firstTimeOnFlashToRAM = false;
  }
  break;

  case '7':
    static bool firstTimeOnBrushless = true;
    if(firstTimeOnBrushless == true)
    {
      for(int i=1; i<brushless_Speed; i++)
      {
        Brushless.write(i);
        delay(25);
      }
      firstTimeOnBrushless = false;
      xTaskCreatePinnedToCore(calculateBrushlessSpeed,"BrushlessSpeed",4000,NULL,1,&BrushlessSpeedHandle,0);
    }
    
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
        if(mapDataList[i].curve == 1)
        {
          mapDataList[i].meanEncoderCount = mapDataList[i].meanEncoderCount + ACCELERATION_OFFSET;
        }
        if (i == 0)
        {
          string meanEncoderCount = to_string(mapDataList[i].meanEncoderCount);
          string curve = to_string(mapDataList[i].curve);
          string curveSpeed = to_string(mapDataList[i].curveSpeed);
          string accelerationCount = to_string(mapDataList[i].accelerationCount);
          string desaccelerationCount = to_string(mapDataList[i].desaccelerationCount);

          string dataString = meanEncoderCount + "," + curve + "," + curveSpeed + "," + accelerationCount + "," + desaccelerationCount + "\n";
          writeFile("/Map_Data.txt", dataString.c_str());
        }
        else
        {
          string meanEncoderCount = to_string(mapDataList[i].meanEncoderCount);
          string curve = to_string(mapDataList[i].curve);
          string curveSpeed = to_string(mapDataList[i].curveSpeed);
          string accelerationCount = to_string(mapDataList[i].accelerationCount);
          string desaccelerationCount = to_string(mapDataList[i].desaccelerationCount);

          string dataString = meanEncoderCount + "," + curve + "," + curveSpeed + "," + accelerationCount + "," + desaccelerationCount + "\n";
          appendFile("/Map_Data.txt", dataString.c_str());
        }
      }
      firstTimeProcess = false;
    }    
    status = '3';
    lastReceivedChar = '3';
  break;

  default:
    analogWrite(in_dir1,255);
    analogWrite(in_dir2,255);

    analogWrite(in_esq1,255);
    analogWrite(in_esq2,255);

    Brushless.write(0);
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
  pinMode(battery, INPUT);

  Serial.begin(115200);
  led_stip.begin();
  I2CMPU6050.begin(I2C_SDA, I2C_SCL, 100000);
  SerialBT.begin("Semreh"); //Bluetooth device name

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_A, enc_eq_B);
  encoder2.attachFullQuad(enc_dir_B, enc_dir_A);

  encoder.clearCount();
  encoder2.clearCount();

  Brushless.attach(BRUSHLESS_PIN, 1000, 2000);

  Brushless.write(180);
  delay(5000);
  Brushless.write(0);

  if(!SPIFFS.begin(true)){
        SerialBT.println("SPIFFS Mount Failed");
        return;
  }
  if (!mpu6050.begin(0x68, &I2CMPU6050)) {
    Serial.println("Failed to find MPU6050 chip");
  }

  sArray.setTypeMCP3008();
  sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
  sArray.setSamplesPerSensor(5);

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
  // SerialBT.print(encoder.getCount());
  // SerialBT.print(" || ");
  // SerialBT.println(encoder2.getCount());
  // SerialBT.println(robotSpeed);
}