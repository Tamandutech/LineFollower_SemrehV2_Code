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

float Ki = 0; // 0.002 
float Kp = 0.043; // 0.04352
float Kd = 0.25; // 0.0992

float KiR = 0;
float KpR = 0.035; // 0.0392
float KdR = 0.0899; // 0.097

void ler_sensores()
{

  uint16_t sArraychannels[sArray.getSensorCount()];
  erro_sensores = sArray.readLineWhite(sArraychannels) - 3500;
  erro_f = -1 * erro_sensores;
 
}
void calcula_PID(float KpParam, float KdParam, float KiParam)
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
  
  PID = (KpParam * P) + (KdParam * D) + (KiParam * I);
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

void calculaEControlePID(int leftSpeed, int rightSpeed) { //Implementar parametro de tipo de PID; reta, curva, curva longa
    calcula_PID(Kp,Kd,Ki);
    controle_motores(leftSpeed, rightSpeed);
}

void controle_com_mapeamento2(int encVal) {

    for (int i = 0; i < numRanges; ++i) {
        if (encVal > ranges[i].minValue && encVal < ranges[i].maxValue) {

            calculaEControlePID(ranges[i].leftMotorSpeed, ranges[i].rightMotorSpeed);

            if (i > numRanges-1) {
                digitalWrite(stby, LOW);
            }
            return;
        }
    }
    // Se nenhum intervalo for correspondido, executar ação padrão
    calcula_PID(Kp,Kd,Ki);
    controle_motores(40,40);
}

//################ FIM: NOVA PROPOSTA DE CONTROLE COM MAPEAMENTO ###############################
//##############################################################################################

void controle_com_mapeamento(int encVal){
  if(encVal > 0 && encVal <= 29000){
    calcula_PID(Kp,Kd,Ki);
    controle_motores(100,100);
  }
  else if(encVal > 29000 && encVal <= 30000){
    calcula_PID(KpR,KdR,KiR);
    controle_motores(245, 245);
  }
  else if(encVal > 30000 && encVal <= 33000){
    calcula_PID(Kp,Kd,Ki);
    controle_motores(100, 100);
  }
  else if(encVal > 33000){
    digitalWrite(stby, LOW);
  }
  else{
    calcula_PID(Kp,Kd,Ki);
    controle_motores(40,40);
  }
  }

void rampa_de_velocidade(uint32_t time) { // implementar a rampa por distancia ao invez de tempo

  //adicionar condicional com sensor lateral esquerdo
  for (int i = 0; i < 100 ; i++){
    controle_motores(i, i);
    delay(time/254);
  }

}

void ler_sens_lat_esq(void * parameter){
  while (1) {
    int inputValue = analogRead(s_lat_esq);
    if (inputValue < 2000) {
      SerialBT.print(encoder.getCount());
      SerialBT.print(",");
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
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 5ms entre as verificações
  }
}

void ler_sens_lat_dir(void * parameter){
  while (1) {
    int inputValue = analogRead(s_lat_dir);
    if (inputValue < 2000) {
      SerialBT.println("INICIO/FIM");
      digitalWrite(buzzer, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));  // Manter o buzzer ligado por 100ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
    }
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(5));  // Pausa de 20ms entre as verificações
  }
}

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
  sArray.setSamplesPerSensor(5); // VERIFICAR SE MUDAR ESSE PARAMETRO MELHORA A LEITURA, QUANTO MAIOR O NUMERO MAIS LENTA A LEITURA POREM MAIS PRECISA

  led_stip.setPixelColor(0, 0, 255, 0);
  led_stip.show();

  for (uint16_t i = 0; i < 300; i++)
  {
    sArray.calibrate();
    delay(20);
  }
  
  xTaskCreatePinnedToCore(ler_sens_lat_esq,"Sensor lat esq",4000,NULL,1,NULL,PRO_CPU_NUM);
  //xTaskCreatePinnedToCore(ler_sens_lat_dir,"Sensor lat dir",4000,NULL,1,NULL,PRO_CPU_NUM);

}
int flag = 0;
void loop()
{
  int inputValue = analogRead(s_lat_dir);
  if (inputValue < 2000 && flag==0) {
    SerialBT.println("INICIO/FIM");
    digitalWrite(buzzer, HIGH);
    encoder2.clearCount();
    encoder.clearCount();
    flag = 1;
  }
  led_stip.setPixelColor(0, 255, 0, 0);
  led_stip.show();

  ler_sensores();
  controle_com_mapeamento((encoder.getCount()+encoder2.getCount())/2);
  //calcula_PID(Kp,Kd,Ki);
  //controle_motores(100,100);
}