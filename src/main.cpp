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

char lastReceivedChar = '4';

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

void controle_com_mapeamento(int encVal)
{
  //começo de pista lateral direito até primeira marcaçao
  if(encVal>0 && encVal<=150) {calcula_PID(Kp,Kd); controle_motores(150);led_stip.setPixelColor(0,B);}
  
  //primeira curva
  else if(encVal>150 && encVal<=760) {calcula_PID(Kp,Kd); controle_motores(100);led_stip.setPixelColor(0,R);}

  //primeira retinha
  else if(encVal>760 && encVal<=1000) {calcula_PID(Kp,Kd); controle_motores(180);led_stip.setPixelColor(0,G);}
  
  //trevo
  else if(encVal>1450 && encVal<=7500) {calcula_PID(Kp,Kd);controle_motores(170);led_stip.setPixelColor(0,B);}

  //curva depois do trevo
  else if(encVal>7500 && encVal<=8200) {calcula_PID(Kp,Kd);controle_motores(145);led_stip.setPixelColor(0,R);}

  //antes do U
  else if(encVal>8200 && encVal<=8900) {calcula_PID(Kp,Kd); controle_motores(150);led_stip.setPixelColor(0,G);}

  //curvinha antes do U
  else if(encVal>8900 && encVal<=9300) {calcula_PID(Kp,Kd); controle_motores(120);led_stip.setPixelColor(0,B);}

  //U
  else if(encVal>9300 && encVal<=11000) {calcula_PID(Kp,Kd); controle_motores(130);led_stip.setPixelColor(0,R);}

  else{calcula_PID(Kp,Kd); controle_motores(110); led_stip.setPixelColor(0,W);}
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
  char semicolon = ';';
  int encoderCountEsq = encoder.getCount();
  int encoderCountDir = encoder2.getCount();

  while(true)
  {
    int inputEsq = analogRead(s_lat_esq);
    
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
    
    if(medLateralEsq < 3000 || medLateralDir < 3000) //lê marcação esquerdo ou direito
    {
      if(medLateralEsq < 2000 && medLateralDir > 3500) //lê apenas marcação esquerda
      {
        SerialBT.print(semicolon); SerialBT.print(encoderCountEsq); SerialBT.print(semicolon); SerialBT.print(encoderCountDir); SerialBT.print(semicolon); SerialBT.println((encoderCountEsq + encoderCountDir)/2);
        led_stip.setPixelColor(1, 0, 0, 255);
        led_stip.show();
        digitalWrite(buzzer, HIGH);  // Ligar o buzzer
        vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
        digitalWrite(buzzer, LOW);   // Desligar o buzzer
        led_stip.setPixelColor(1, 0, 0, 0);
        led_stip.show();
      }

      else if(medLateralEsq > 3500 && medLateralDir < 2000) //lê apenas marcação direita
      {
        tempo = millis();
        SerialBT.println("INICIO/FIM");
        encoder2.clearCount();
        encoder.clearCount();

        digitalWrite(buzzer, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
        digitalWrite(buzzer, LOW);   // Desligar o buzzer
      }
    }
    medLateralDir = 0;
    medLateralEsq = 0;

    countLateral = countLateral % MED_TAMANHO;
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 5ms entre as verificações
  }
}

void callRobotTask(char status)
{
  switch(status)
  {
  case '0': //Map
      ler_sensores();
      calcula_PID(Kp,Kd);
      controle_motores(MAPPING_PWM);
  break;
  
  case '1': //Propeller
      analogWrite(PROPELLER_PIN,PROPELLER_PWM);
  break;

  case '2': //Run
      ler_sensores();
      calcula_PID(Kp,Kd);
      controle_motores(HIGHSPEED_PWM);
  break;

  case '3': //Abort
      digitalWrite(stby, LOW);
      analogWrite(PROPELLER_PIN, 0);
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

  led_stip.setPixelColor(0, 0, 0, 255);
  led_stip.show();

  for (uint16_t i = 0; i < 300; i++)
  {
    sArray.calibrate();
    delay(20);
  }
  
  //xTaskCreatePinnedToCore(ler_sens_lat_esq,"Sensor lat esq",4000,NULL,1,NULL,0);
  //xTaskCreatePinnedToCore(ler_sens_lat_dir,"Sensor lat dir",4000,NULL,1,NULL,0);
  xTaskCreatePinnedToCore(ler_laterais,"Sensores Laterais",4000,NULL,1,NULL,0);
}

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

void loop()
{  
  bluetoothRead();

  callRobotTask(lastReceivedChar);

  led_stip.setPixelColor(0, 0, 255, 0);
  led_stip.show();
}