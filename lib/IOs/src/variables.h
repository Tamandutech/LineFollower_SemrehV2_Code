//Botão BOOT
#define boot 0

//Ventoinha
#define PROPELLER_PIN 5
#define BRUSHLESS_PIN 5

//ADC sensores frontais
#define out_s_front 19
#define clk 18
#define in_s_front 22
#define cs_s_front 23

//driver motor
#define PWM_LEFT 13 // Pino PWM do motor esquerdo
#define PWM_RIGHT 14 // Pino PWM do motor direito

#define in_esq2	21 // Pino input1 motor esquerdo
#define in_esq1 25 // Pino input2 motor esquerdo

#define in_dir1 26 // Pino input1 motor direito
#define in_dir2 27 // Pino input2 motor direito

#define stby 17 // Pino stand-by dos motores

//Sensores laterais
#define s_lat_esq 39 //39 // Pino sensor lateral esquerdo
#define s_lat_dir 33 //33 // Pino sensor lateral direito

//Encoder
#define enc_eq_A 35 // Pino inputA encoder esquerdo
#define enc_eq_B 34 // Pino inputB encoder esquerdo
#define enc_dir_A 4 // Pino inputA encoder direito
#define enc_dir_B 16 // Pino inputB encoder direito

//Alertas
#define led 32 // Pino dos LEDs

//Numero de LEDs NeoPixels na placa
#define LED_COUNT 2 // Numero de LEDs

//Valores para os motores
int leftEncoderPulse = 0;
int rightEncoderPulse = 0;
int leftDistanceTravelled = 0;
int rightDistanceTravelled = 0;
float robotSpeed = 0;
float last_pwm = 0;
float run_pwm;
float translacionalError;
float lastTranslacionalError;
float P_Translacional = 0;
float D_Translacional = 0;
float I_Translacional = 0;
float PIDTranslacional = 0;

float rotacionalError;
float lastRotacionalError;
#define MAX_PWM 255

#define HIGHSPEED_PWM 130

#define MAPPING_PWM 180

#define PROPELLER_PWM 200

#define BRUSHLESSSPEED 145   //100graus = 235g   125graus = 300g    150graus = 385g
#define BRUSHLESSSPEED2 165

//Valores do PID
float Kp = 0.0123; // 0.074  M120 Curva
float Kd = 0.15; //  0.48   M120 Curva

float KpR = 0.05; // M255
float KdR = 0.15; //  M255

float P = 0, D = 0; // Valores de ganho do PID
float PID = 0; // Valor do ganho do PID total

float KpTrans = 1;
float KdTrans = 20;
float KiTrans = 4.6;

float KpRot=7;
float KdRot=0;
float KiRot=10;

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
float velesqTranslacional = 0, veldirTranslacional = 0;
float erro_sensores = 0; // Erro dos sensores (-3500 < x < 3500)
float erro_anterior = 0; // Erro anterior dos sensores (-3500 < x < 3500)
float erro_f = 0; // Erro dos sensores (-3500 < x < 3500)



//variáveis globais controle rotacional
long int enc_esq_pul = 0;
long int enc_dir_pul = 0;
long int pul_prev_esq = 0;
long int pul_prev_dir = 0;

float erro_f_rot = 0;
float erro_anterior_rot = 0;
float P_rot = 0 ;
float D_rot = 0;
float I_rot = 0;
float PIDrot = 0;

float MM_PER_COUNT = 0.576f;
#define SAMPLING_TIME 10

float DISTANCEWHEELTOCENTER = 0.126f;
#define mmPerPulse 0.576
float GRAVITY = 9.8f;
float FRICTION = 0.577f;
//#define desaceleracao 5
float MASS = 0.172f;
float BRUSHLESSFORCE = 3.0f;
float MAXSPEED = 4.0f;
float MAXSPEED2 = 4.5f;
float acceleration = 15.0f;

#define FORMAT_LITTLEFS_IF_FAILED true

#define ACCELERATION_OFFSET 350
