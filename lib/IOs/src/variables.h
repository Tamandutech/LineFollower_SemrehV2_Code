//ADC sensores frontais
#define out_s_front 19
#define clk 18
#define in_s_front 22
#define cs_s_front 23

//driver motor
#define pwmA 13 // Pino PWM do motor esquerdo
#define pwmB 14 // Pino PWM do motor direito

#define in_esq1	25 // Pino input1 motor esquerdo
#define in_esq2 21 // Pino input2 motor esquerdo

#define in_dir1 26 // Pino input1 motor direito
#define in_dir2 27 // Pino input2 motor direito

#define stby 17 // Pino stand-by dos motores

//Sensores laterais
#define s_lat_esq 39 // Pino sensor lateral esquerdo
#define s_lat_dir 33 // Pino sensor lateral direito

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

//Valores do PID
float I = 0, P = 0, D = 0; // Valores de ganho do PID
float PID = 0; // Valor do ganho do PID total

//Dados para integral
int error1=0, error2=0, error3=0, error4=0, error5=0, error6=0; // Erros para o calculo da integral

float velesq = 0, veldir = 0; // Valor de PWM do motor
float erro_sensores = 0; // Erro dos sensores (-3500 < x < 3500)
float erro_anterior = 0; // Erro anterior dos sensores (-3500 < x < 3500)
float erro_f = 0; // Erro dos sensores (-3500 < x < 3500)