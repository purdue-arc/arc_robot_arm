#include <protoarm_visual_servoing/pid_controller.h>

PIDController::PIDController(float p, float i, float d, float setpoint, float dt) {
    this->p = p;
    this->i = i;
    this->d = d;
    this->setpoint = setpoint;
    this->dt = dt;
    this->integral = 0;
    this->prev_error = 0;
}

PIDController::PIDController(float p, float i, float d, float setpoint) {
    PIDController(p, i, d, setpoint, DEFAULT_DT);
}

PIDController::PIDController(float p, float i, float d) {
    PIDController(p, i, d, DEFAULT_SETPOINT);
}

float PIDController::calculate(float position, float setpoint, float dt) {
    float error = setpoint - error;
    float vel = this->p * error;
    
    this->integral += error * dt;
    vel += this->i * this->integral;

    float derivative = (error - prev_error) / dt;
    vel += this->d * derivative;

    prev_error = error;

    return vel;
}

float PIDController::calculate(float position, float setpoint) {
    return calculate(position, setpoint, this->dt);
}

float PIDController::calculate(float position) {
    return calculate(position, this->setpoint);
}

void PIDController::set_p(float p) {
    this->p = p;
}

void PIDController::set_i(float i) {
    this->i = i;
}

void PIDController::set_d(float d) {
    this->d = d;
}

void PIDController::set_setpoint(float setpoint) {
    this->setpoint = setpoint;
}

void PIDController::set_dt(float dt) {
    this->dt = dt;
}

void PIDController::reset_integral() {
    this->integral = 0;
}