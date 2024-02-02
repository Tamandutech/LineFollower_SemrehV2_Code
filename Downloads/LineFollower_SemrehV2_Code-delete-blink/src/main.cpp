#include <Arduino.h>
#include <ESP32Encoder.h>
#include <variables.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

//tempo recorde 12150

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;
BluetoothSerial SerialBT;
Adafruit_NeoPixel led_stip(LED_COUNT, led, NEO_GRB + NEO_KHZ800); // Declare our NeoPixel strip object

float Ki = 0; // 0.002 
float Kp = 0.043; // 0.074  M120 Curva
float Kd = 0.25; //  0.48   M120 Curva

float KiR = 0;
float KpR = 0.042; // M255
float KdR = 0.53; //  M255

const uint32_t R = led_stip.Color(255,0,0);
const uint32_t G = led_stip.Color(0,255,0);
const uint32_t B = led_stip.Color(0,0,255);
const uint32_t W = led_stip.Color(255,255,255);

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
  I = error1 + error2 + error3 + error4 + error5 + error6;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = P;
  
  PID = (KpParam * P) + (KdParam * D) + (Ki * I);
  erro_anterior = erro_f;
}
void controle_motores(float vel_A, float vel_B)
{
  velesq = vel_A + PID;
  veldir = vel_B - PID;

  if(veldir>=0)
  {
    if(veldir > 255) veldir = 255;
    digitalWrite(in_dir1,LOW);
    digitalWrite(in_dir2,HIGH);
  }
  else
  {
    veldir = (-1) * veldir;
    digitalWrite(in_dir1,HIGH);
    digitalWrite(in_dir2,LOW);
  }
  analogWrite(pwmB,veldir);

  if(velesq>=0)
  {
    if (velesq > 255) velesq = 255;
    digitalWrite(in_esq1,LOW);
    digitalWrite(in_esq2,HIGH);
  }
  else
  {
    velesq = (-1) * velesq;
    digitalWrite(in_esq1,HIGH);
    digitalWrite(in_esq2,LOW);
  }
  analogWrite(pwmA,velesq);
}


//##############################################################################################
//################ INICIO: NOVA PROPOSTA DE CONTROLE COM MAPEAMENTO ############################

struct Range {
    char estado;
    int minValue;
    int maxValue;
    int leftMotorSpeed;
    int rightMotorSpeed;
};

const int numRanges = 5;  // Número de intervalos

Range ranges[numRanges] = {
    {'c',0, 155, 170, 170},         // começo da pista
    {'c',760, 1000, 180, 180},    // primeira retinha
    {'c',1450, 7000, 175, 175},   // trevo
    {'c',7001, 12000, 140, 140},  // parte superior da pista
    {'s',23000, INT_MAX, 0, 0},   // standby
    // {178000, 207900, 240, 240},   // intervalo 5
    // {221000, 280000, 255, 255},   // intervalo 7
    // {275000, INT_MAX, 120, 120}   // final
};

void controle_com_mapeamento2(int encVal) {

    for (int i = 0; i < numRanges; ++i) {
        if (encVal >= ranges[i].minValue && encVal < ranges[i].maxValue) {
          if(ranges[i].estado == 'r')
          {
            calcula_PID(KpR,KdR);
            controle_motores(ranges[i].leftMotorSpeed, ranges[i].rightMotorSpeed);
          }
          else if(ranges[i].estado == 'c')
          {
            calcula_PID(Kp,Kd);
            controle_motores(ranges[i].leftMotorSpeed, ranges[i].rightMotorSpeed);
          }
          else if (ranges[i].estado == 's') {
              digitalWrite(stby, LOW);
          }
          else
          {
            calcula_PID(Kp,Kd);
            controle_motores(110, 110);
          }
            return;
        }
    }
}

//################ FIM: NOVA PROPOSTA DE CONTROLE COM MAPEAMENTO ###############################
//##############################################################################################
int flager = 0;
u_int64_t tempo;
void controle_com_mapeamento(int encVal){

  if (flager == 0)
  {
    //começo de pista lateral direito até primeira marcaçao
    if(encVal>0 && encVal<=150) {calcula_PID(Kp,Kd); controle_motores(150,150);led_stip.setPixelColor(0,B);}
    
    //primeira curva
    else if(encVal>150 && encVal<=760) {calcula_PID(Kp,Kd); controle_motores(100,100);led_stip.setPixelColor(0,R);}

    //primeira retinha
    else if(encVal>760 && encVal<=1000) {calcula_PID(Kp,Kd); controle_motores(180,180);led_stip.setPixelColor(0,G);}
    
    //trevo
    else if(encVal>1450 && encVal<=7500) {calcula_PID(Kp,Kd);controle_motores(170,170);led_stip.setPixelColor(0,B);}

    //curva depois do trevo
    else if(encVal>7500 && encVal<=8200) {calcula_PID(Kp,Kd);controle_motores(145,145);led_stip.setPixelColor(0,R);}

    //antes do U
    else if(encVal>8200 && encVal<=8900) {calcula_PID(Kp,Kd); controle_motores(150,150);led_stip.setPixelColor(0,G);}

    //curvinha antes do U
    else if(encVal>8900 && encVal<=9300) {calcula_PID(Kp,Kd); controle_motores(120,120);led_stip.setPixelColor(0,B);}

    //U
    else if(encVal>9300 && encVal<=11000) {calcula_PID(Kp,Kd); controle_motores(130,130);led_stip.setPixelColor(0,R);}

    //pós U
    else if(encVal>11600) {calcula_PID(Kp,Kd); controle_motores(155,155);led_stip.setPixelColor(0,G);
      int inputValue = analogRead(s_lat_esq);
      if (inputValue < 3000) {
        encoder.clearCount();
        encoder2.clearCount();
        flager = 1;
        digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      }
    }
    //toda a pista
    else{
      digitalWrite(buzzer, LOW);  // Ligar o buzzer
      calcula_PID(Kp,Kd);
      controle_motores(110,110);
      led_stip.setPixelColor(0,W);
    }

  }
  else //pós flagger
  {
    //CURVONA
    if(encVal<=1350) {calcula_PID(Kp,Kd);controle_motores(170,170);led_stip.setPixelColor(0,B);}

    //PRIMEIRA RETA CRUZADA
    else if(encVal>1350 && encVal<=1900) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,R);digitalWrite(buzzer,HIGH);}
    //FINAL PRIMEIRA RETA CRUZADA
    else if(encVal >1900 && encVal<=2100) {calcula_PID(Kp,Kd); controle_motores(140,140);led_stip.setPixelColor(0,G);}
    //CURVA
    else if(encVal >2100 && encVal<=3250) {calcula_PID(Kp,Kd); controle_motores(110,110);led_stip.setPixelColor(0,B);digitalWrite(buzzer,LOW);}

    //SEGUNDA RETA CRUZADA
    else if(encVal >3250 && encVal<=3970) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,R);digitalWrite(buzzer,HIGH);}
    
    //CURVA
    else if(encVal >3970 && encVal<=5120) {calcula_PID(Kp,Kd); controle_motores(110,110);led_stip.setPixelColor(0,G);digitalWrite(buzzer,LOW);}

    //TERCEIRA RETA CRUZADA
    else if(encVal >5120 && encVal<=5300) {calcula_PID(Kp,Kd); controle_motores(145,145);led_stip.setPixelColor(0,B);digitalWrite(buzzer,HIGH);}

    //CURVA
    else if(encVal >5300 && encVal<=6400) {calcula_PID(Kp,Kd); controle_motores(110,110);led_stip.setPixelColor(0,R);digitalWrite(buzzer,LOW);}

    //QUARTA RETA CRUZADA
    else if(encVal >6400 && encVal<=6600) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,G);digitalWrite(buzzer,HIGH);}
    
    //FINAL QUARTA RETA CRUZADA
    else if(encVal >6600 && encVal<=6900) {calcula_PID(Kp,Kd); controle_motores(170,170);led_stip.setPixelColor(0,B);}
    //CURVA
    else if(encVal >6900 && encVal<=8200) {calcula_PID(Kp,Kd); controle_motores(110,110);led_stip.setPixelColor(0,R);digitalWrite(buzzer,LOW);}

    //QUINTA RETA
    else if(encVal >8200 && encVal<=8900) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,G);digitalWrite(buzzer,HIGH);}
    //## aumentei de 8600 para 8900

    //TRIANGULO
    else if(encVal >8900 && encVal<=10600) {calcula_PID(Kp,Kd); controle_motores(110,110);led_stip.setPixelColor(0,B);digitalWrite(buzzer,LOW);}

    //Reta pos triangulo
    else if(encVal >10600 && encVal<=11200) {calcula_PID(Kp,Kd); controle_motores(170,170);led_stip.setPixelColor(0,B);digitalWrite(buzzer,LOW);}

    //POS TRIANGULO
    else if(encVal >11200 && encVal<=12900) {calcula_PID(Kp,Kd); controle_motores(120,120);led_stip.setPixelColor(0,R);}

    //Reta fim de pista
    else if(encVal >12900 && encVal<=13500) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,G);}

    //FIM DE PISTA
    else if(encVal >13500 && encVal<=13700) {calcula_PID(KpR,KdR); controle_motores(250,250);led_stip.setPixelColor(0,G);
    int inputValue = analogRead(s_lat_dir);
      if (inputValue < 3000) {
        tempo = millis() - tempo;
        SerialBT.println(tempo);
        digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      }
    }

    else if(encVal >13700 ) {digitalWrite(stby,LOW);}

    else {calcula_PID(Kp,Kd); controle_motores(100,100);led_stip.setPixelColor(0,W);}
  }
}

void rampa_de_velocidade(uint32_t time) { // implementar a rampa por distancia ao invez de tempo

  //adicionar condicional com sensor lateral esquerdo
  for (int i = 0; i < 100 ; i++){
    controle_motores(i, i);
    delay(time/254);
  }

}

//int64_t encoderCount;
//int64_t encoderCount2;
//int64_t encoderMed;


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
      SerialBT.println("INICIO/FIM");

      digitalWrite(buzzer, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
    }
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 20ms entre as verificações
  }
}

void ler_laterais(void *parameter){
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
    if(medLateralEsq < 3000 || medLateralDir < 3000)
    {
    if(medLateralEsq < 2000 && medLateralDir > 3500)
    {
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
    }
    medLateralDir = 0;
    medLateralEsq = 0;

    countLateral = countLateral % MED_TAMANHO;
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 20ms entre as verificações

  }
}
char comando_bt;
void controle_bt(char ComandoBt){
  comando_bt = '0';
  comando_bt = (char)SerialBT.read();
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  
  }
  if (SerialBT.available()) {
    if(comando_bt=='1'){
      while(true){
        digitalWrite(stby,LOW);
        analogWrite(vento,0);
      }
    }
    else if(comando_bt == '2'){
      analogWrite(vento,60);
    }
  }
}
int t1;
void setup()
{
  Serial.begin(115200);

  pinMode(in_esq1, OUTPUT);
  pinMode(in_esq2, OUTPUT);
  pinMode(in_dir1, OUTPUT);
  pinMode(in_dir2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(s_lat_esq, INPUT);
  pinMode(s_lat_dir, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(vento, OUTPUT);
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

  t1 = millis();
}

int flag = 0;
int botao;
int botao2 = 0;

void loop()
{  
  controle_bt(comando_bt);
  int inputValue = analogRead(s_lat_dir);
  if (inputValue < 3000 && flag==0) {
    tempo = millis();
    SerialBT.println("INICIO/FIM");
    encoder2.clearCount();
    encoder.clearCount();
    flag = 1;
  } 
  led_stip.setPixelColor(0, 0, 255, 0);
  led_stip.show();
  ler_sensores();
  calcula_PID(Kp,Kd);
  //controle_com_mapeamento((encoder.getCount()+encoder2.getCount())/2);
  controle_motores(100,100);
}