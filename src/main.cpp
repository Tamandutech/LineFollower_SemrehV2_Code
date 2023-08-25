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
Adafruit_NeoPixel led_stip(LED_COUNT, led, NEO_GRB + NEO_KHZ800); // Declare our NeoPixel strip object

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

  digitalWrite(in_dir1,HIGH);
  digitalWrite(in_dir2,LOW);
  analogWrite(pwmA,veldir);

  digitalWrite(in_esq1,HIGH);
  digitalWrite(in_esq2,LOW);
  analogWrite(pwmB,velesq);
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
    calcula_PID();
    controle_motores(leftSpeed, rightSpeed);
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
    calcula_PID();
    controle_motores(40,40);
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
        calcula_PID();
        controle_motores(245, 245);
      }      else if(encVal > 109850 && encVal < 114600){ //antes reta media
        digitalWrite(buzzer, HIGH);
        calcula_PID();
        controle_motores(245, 245);

      }else if(encVal > 119600 && encVal < 137000){ //reta media
        digitalWrite(buzzer, HIGH);
        calcula_PID();
        controle_motores(255, 255);
      }
      else if(encVal > 144700 && encVal < 149000){ //reta vertical
        digitalWrite(buzzer, HIGH);

        calcula_PID();
        controle_motores(245, 245);
      }  
      else if(encVal > 178000 && encVal < 183300){ //reta 1 de 3
          digitalWrite(buzzer, HIGH);

          calcula_PID();
          controle_motores(240, 240);
      }
      else if(encVal > 190000 && encVal < 195400){ //reta 2 de 3
          digitalWrite(buzzer, HIGH);

          calcula_PID();
          controle_motores(240, 240);
      }
      else if(encVal > 202200 && encVal < 207900){ //reta 3 de 3
        digitalWrite(buzzer, HIGH);

        calcula_PID();
        controle_motores(240, 240);
      }
      else if(encVal > 221000 && encVal < 251000){ //retona
        digitalWrite(buzzer, HIGH);

        calcula_PID();
        controle_motores(255, 255);
      } 
      else if(encVal > 250800 && encVal < 280000){ //final
        calcula_PID();
        controle_motores(120,120);
      }
      else if(encVal > 275000){
        digitalWrite(stby, LOW);
      }
      else{
        calcula_PID();
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

int v = 0;

void ler_sens_lat_esq(void * parameter){
  while (1) {
    int inputValue = analogRead(s_lat_esq);
    if (inputValue < 200) {
      digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(250));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, LOW);  // Ligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(50));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(250));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(50));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, HIGH);   // Desligar o buzzer}
      vTaskDelay(pdMS_TO_TICKS(500));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer}
    
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(20));  // Pausa de 100ms entre as verificações
    }
  }
}

void ler_sens_lat_dir(void * parameter){
  while (1) {
    int inputValue = analogRead(s_lat_dir);
    if (inputValue < 200) {

      SerialBT.print(encoder.getCount());
      SerialBT.print(",");
      SerialBT.println(encoder2.getCount());

      led_stip.setPixelColor(1, 0, 0, 255);
      led_stip.show();
      digitalWrite(buzzer, HIGH);  // Ligar o buzzer
      vTaskDelay(pdMS_TO_TICKS(200));  // Manter o buzzer ligado por 500ms
      digitalWrite(buzzer, LOW);   // Desligar o buzzer
      led_stip.setPixelColor(1, 0, 0, 0);
      led_stip.show();
    }
    // Pequeno atraso para evitar detecção repetida muito rápida
    vTaskDelay(pdMS_TO_TICKS(20));  // Pausa de 100ms entre as verificações
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

  led_stip.setPixelColor(0, 0, 255, 0);
  led_stip.show();

  for (uint16_t i = 0; i < 300; i++)
  {
    sArray.calibrate();
    delay(20);
  }
  
  xTaskCreate(ler_sens_lat_esq,"Sensor lat esq",1000,NULL,1,NULL);
  xTaskCreate(ler_sens_lat_dir,"Sensor lat dir",1000,NULL,1,NULL);

}

void loop()
{
  led_stip.setPixelColor(0, 255, 0, 0);
  led_stip.show();

  ler_sensores();
  calcula_PID();
  controle_motores(40,40);
}