//Botão BOOT
#define boot 0

//Ventoinha
#define PROPELLER_PIN 5

//ADC sensores frontais
#define out_s_front 19
#define clk 18
#define in_s_front 22
#define cs_s_front 23

//driver motor
#define PWM_LEFT 13 // Pino PWM do motor esquerdo
#define PWM_RIGHT 14 // Pino PWM do motor direito

#define in_esq2	25 // Pino input1 motor esquerdo
#define in_esq1 21 // Pino input2 motor esquerdo

#define in_dir1 26 // Pino input1 motor direito
#define in_dir2 27 // Pino input2 motor direito

#define stby 17 // Pino stand-by dos motores

//Sensores laterais
#define s_lat_esq 39 //39 // Pino sensor lateral esquerdo
#define s_lat_dir 33 //33 // Pino sensor lateral direito

//Encoder
#define enc_eq_A 34 // Pino inputA encoder esquerdo
#define enc_eq_B 35 // Pino inputB encoder esquerdo
#define enc_dir_A 16 // Pino inputA encoder direito
#define enc_dir_B 4 // Pino inputB encoder direito

//Alertas
#define led 32 // Pino dos LEDs
#define buzzer 12 // Pino buzzer

//Numero de LEDs NeoPixels na placa
#define LED_COUNT 2 // Numero de LEDs

//Valores para os motores
#define MAX_PWM 255

#define HIGHSPEED_PWM 130

#define MAPPING_PWM 70

#define PROPELLER_PWM 200

//Valores do PID
float Kp = 0.043; // 0.074  M120 Curva
float Kd = 0.25; //  0.48   M120 Curva

float KpR = 0.043; // M255
float KdR = 0.43; //  M255

float P = 0, D = 0; // Valores de ganho do PID
float PID = 0; // Valor do ganho do PID total

//Valores para leitura do sensores laterais
#define DEBOUNCETIME 200
#define MED_TAMANHO 3
int countLateral = 0; 
float medLateralEsq = 0;
float medLateralDir = 0;

float accumLateralDir[MED_TAMANHO] = {};
float accumLateralEsq[MED_TAMANHO] = {};

float velesq = 0, veldir = 0; // Valor de PWM do motor
float velesqrot = 0, veldirrot = 0;
float erro_sensores = 0; // Erro dos sensores (-3500 < x < 3500)
float erro_anterior = 0; // Erro anterior dos sensores (-3500 < x < 3500)
float erro_f = 0; // Erro dos sensores (-3500 < x < 3500)



// variáveis globais controle rotacional
long int enc_esq_pul = 0;
long int enc_dir_pul = 0;
long int pul_prev_esq = 0;
long int pul_prev_dir = 0;

float erro_f_rot = 0;
float erro_anterior_rot = 0;
float P_rot = 0 ;
float D_rot = 0;
float I_rot = 0;
float KpParamRot=180;
float PIDrot = 0;
float KdParamRot=3;
float KiParamRot=10;

