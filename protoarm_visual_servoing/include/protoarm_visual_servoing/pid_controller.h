#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define DEFAULT_DT 0.02
#define DEFAULT_SETPOINT 0

class PIDController {
private:
    float p;
    float i;
    float d;
    float setpoint;
    float dt;
    float prev_error;
    float integral;
public:
    PIDController() {};
    PIDController(float p, float i, float d);
    PIDController(float p, float i, float d, float setpoint);
    PIDController(float p, float i, float d, float setpoint, float dt);
    float calculate(float position);
    float calculate(float position, float setpoint);
    float calculate(float position, float setpoint, float dt);
    void set_setpoint(float setpoint);
    void set_dt(float dt);
    void set_p(float p);
    void set_i(float i);
    void set_d(float d);
    void reset_integral();
};

#endif