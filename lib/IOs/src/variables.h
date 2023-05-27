//ADC_sensores_frontais
#define out_s_front 19
#define clk 18
#define in_s_front 22
#define cs_s_front 23

//driver_motor
#define pwmA 14
#define pwmB 13

#define in_dir1	25
#define in_dir2 21

#define in_esq1 26
#define in_esq2 27

#define stby 17

//sensores_laterais
#define s_lat_esq 39
#define s_lat_dir 33

//encoder
#define enc_eq_A 34
#define enc_eq_B 35
#define enc_dir_A 16
#define enc_dir_B 4

//Alerts
#define led 32
#define buzzer 12

const float pul_per_turn = 360;
int count_enc_esq = 0;
int count_enc_dir = 0;
int rpm_me = 0;
int rpm_md = 0;

int timer_in = 0;
int timer_prev = 0;
int timer_prev3 = 0;
long int pul_prev_eq = 0;
long int pul_prev_dir = 0;
long int enc_esq_pul;
long int enc_dir_pul;


float I = 0, P = 0, D = 0, PID = 0;
float IR = 0, PR = 0, DR = 0, PIDR = 0;

float velesq = 0, veldir = 0, velesqR = 0, veldirR = 0;
float erro_sensores = 0, erro_anterior = 0;
float erro_f = 0;
int linha_l = 0;
int timer_prev2 = 0;
int timer_prev4 = 0;



