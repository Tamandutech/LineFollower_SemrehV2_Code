//ADC_sensores_frontais
#define out_s_front 32
#define clk 33
#define in_s_front 25
#define cs_s_front 26

//driver_motor
#define pwmA 2
#define pwmB 23

#define in_dir1	16
#define in_dir2 4

#define in_esq1 21
#define in_esq2 22

#define stby 17

//sensores_laterais
#define s_lat_esq 27
#define s_lat_dir 35

//encoder
#define enc_eq_A 15
#define enc_eq_B 5
#define enc_dir_A 19
#define enc_dir_B 18

#define led 13
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



