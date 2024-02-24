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

void controle_motores(float motorPWM)
{
  velesq = motorPWM + PID;
  veldir = motorPWM - PID;

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

struct MotorControlData {
    int encoderValueBegin;
    int encoderValueEnd;
    int motorPWM;
};

std::vector<MotorControlData> trackMap = {
  //{encoderValueBegin, encoderValueEnd, motorPWM},
    {0, 150, 150},       // Começo de pista lateral direito até primeira marcação
    {151, 760, 100},     // Primeira curva
    {761, 1000, 180},    // Primeira retinha
    {1451, 7500, 170},   // Trevo
    {7501, 8200, 170},   // Curva depois do trevo
    {8201, 8900, 170},   // Antes do U
    {8901, 9300, 170},   // Curvinha antes do U
    {9301, 11000, 170}  // U
};

void motorControlWithMap(int encVal)
{
  bool not_mapped = true;
  for (const auto& MotorControlData : trackMap)
  {
    int firstEncoderValue = MotorControlData.encoderValueBegin;
    int secondEncoderValue = MotorControlData.encoderValueEnd;
    int pwmDeliveredToMotors = MotorControlData.motorPWM;

    if (encVal >= firstEncoderValue && encVal <= secondEncoderValue)
    {
      calcula_PID(Kp, Kd);
      controle_motores(pwmDeliveredToMotors);
      not_mapped = false;
      //return;
    }
  }
  if(not_mapped){
    calcula_PID(Kp, Kd);
    controle_motores(110);
  }
}

void controle_com_mapeamento(int encVal)
{
  //começo de pista lateral direito até primeira marcaçao
  if(encVal<650) {calcula_PID(Kp,Kd); controle_motores(100);}
  //else if(encVal >= 20500 && encVal <= 21700) {calcula_PID(Kp,Kd); controle_motores(255);}
  else if (encVal >= 22000 && encVal <= 23800) {calcula_PID(Kp,Kd); controle_motores(255);}
  else if(encVal>27800 /*&& encVal<=20000*/) {analogWrite(PWM_LEFT,0); analogWrite(PWM_RIGHT,0); analogWrite(PROPELLER_PIN, 0);/*calcula_PID(KpR,KdR); controle_motores(200);led_stip.setPixelColor(0,B);*/}
  
  else{calcula_PID(Kp,Kd); controle_motores(120); led_stip.setPixelColor(0,W);}
}

void ler_sens_lat_esq(void *parameter){
  while (1) {
    int inputValue = analogRead(s_lat_esq);
    if (inputValue < 3000) {
      //encoderCount = encoder.getCount();
      //encoderCount2 = encoder2.getCount();
      //encoderMed = (encoderCount+encoderCount2)/2;

      //SerialBT.printf("; %d ; %d ; %d\n",encoderCount,encoderCount2,encoderMed);
      SerialBT.print(";");
      SerialBT.print(encoder.getCount()); 
      SerialBT.print(";");
      SerialBT.print(encoder2.getCount());
      SerialBT.print(";");
      SerialBT.println((encoder.getCount()+encoder2.getCount())/2);

      led_stip.setPixelColor(1, 0, 0, 255);
      led_stip.show();
      digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
      led_stip.setPixelColor(1, 0, 0, 0);
      led_stip.show();
    }
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 20ms entre as verificações    
  }
}

void ler_sens_lat_dir(void *parameter){
  while (1) {
    int inputValue = analogRead(s_lat_dir);
    if (inputValue < 3000) {
      tempo = millis();
      SerialBT.println("INICIO/FIM");
      encoder2.clearCount();
      encoder.clearCount();

      digitalWrite(buzzer, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
    }
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 20ms entre as verificações
  }
}

void ler_laterais(void *parameter){
  static bool readingWhiteLeft;
  static bool readingWhiteRight;
  static bool firstTimeRight = false;
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
  // case '0': //Calibrate
  //   if (FirstTimeOnSwitchCase == true)
  //   {
  //     for (int i = 0; i < 300; i++)
  //     {
  //       if(i < 200)
  //       {
  //         led_stip.setPixelColor(0,B);
  //         led_stip.show();
  //       }

  //       else
  //       {
  //         led_stip.setPixelColor(0,R);
  //         led_stip.show();
  //       }
  //       sArray.calibrate();
  //       delay(20);
  //     }

  //     led_stip.setPixelColor(0, 0, 0, 0);
  //     led_stip.show();
  //     FirstTimeOnSwitchCase = false;
  //   }
  // break;

  case '1': //Map
    ler_sensores();
    calcula_PID(Kp,Kd);
    controle_motores(MAPPING_PWM);
  break;

  case '2': //Run with track map
    ler_sensores();
    controle_com_mapeamento((encoder.getCount()+encoder2.getCount())/2);
  break;

  case '3': //Abort
    analogWrite(PWM_LEFT,0);
    analogWrite(PWM_RIGHT,0);
    analogWrite(PROPELLER_PIN, 0);
  break;

  case '6': //Propeller
    analogWrite(PROPELLER_PIN,PROPELLER_PWM);
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

  for (int i = 0; i < 300; i++)
  {
    if(i < 200)
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
  // ler_sensores();
  // calcula_PID(KpR,KdR);
  // controle_motores(180);
  bluetoothRead();

  callRobotTask(lastReceivedChar);

  // SerialBT.print(analogRead(s_lat_dir));
  // SerialBT.print(";");
  // SerialBT.println(analogRead(s_lat_esq));
  // delay(300);
}