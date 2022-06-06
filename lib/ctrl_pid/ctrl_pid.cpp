#ifndef CTRL_PID_H
#define CTRL_PID_H

class PID {
    float Kp;    
    float Ki;
    float Kd;

    float P; //ta dando problema o _P ???
    float I; //sempre vai ser 0
    float D;

    float erro;
    float erro_ant;
    float PID_value;

    public:
        PID(float Kp, float Ki, float Kd, float P, float I, float D, float erro, float erro_ant);
        float calculo_pid(float Kp, float Ki, float Kd, float P, float I, float D, float erro, float erro_ant);
};

#endif 