#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

const double PI = 3.1415926535897;
const double WHEEL_DISTANCE = 0.5;
const double WHEEL_RADIUS = 0.1;

class PID {
private:
    double Kp, Ki, Kd, dt;
    double integral = 0;
    double previous_error = 0;

public:
    PID(double kp, double ki, double kd, double t_step) 
        : Kp(kp), Ki(ki), Kd(kd), dt(t_step) {}

    double calculate(double target, double current) {
        double error = target - current;
        double P = Kp * error;
        integral += error * dt;
        double I = Ki * integral;
        double D = Kd * (error - previous_error) / dt;
        previous_error = error;
        return P + I + D;
    }
};

class Motor {
private:
    double current_omega;
    double dt;

public:
    Motor(double time_step) : dt(time_step), current_omega(0) {}

    void act(double omega) { current_omega = omega; }
    
    double getOmega() {  return current_omega / 2; }
    
};

class Robot {
private:
    Motor leftMotor, rightMotor;
    double dt;
    
public:
    Robot(double time_step) : dt(time_step), leftMotor(time_step), rightMotor(time_step) {}
    
    void apply(double v_x, double omega)
    {    
        double v_left  = v_x - (omega * WHEEL_DISTANCE) / 2;
        double v_right = v_x + (omega * WHEEL_DISTANCE) / 2;
        
        double omega_left  = v_left  / WHEEL_RADIUS;
        double omega_right = v_right / WHEEL_RADIUS;
        
        leftMotor.act(omega_left);
        rightMotor.act(omega_right);
    }
    
    void getVelocities(double &V_x, double &omega_z) {
        double v_left  = leftMotor.getOmega()  * WHEEL_RADIUS;
        double v_right = rightMotor.getOmega() * WHEEL_RADIUS;
        
        V_x = (v_left + v_right) / 2;
        omega_z = (v_right - v_left) / WHEEL_DISTANCE;
    }
    
    void getMotorData(double &V_left, double &V_right, double &Omega_left, double &Omega_right) {
        V_left  = leftMotor.getOmega()  * WHEEL_RADIUS;
        V_right = rightMotor.getOmega() * WHEEL_RADIUS;
        Omega_left = leftMotor.getOmega();
        Omega_right = rightMotor.getOmega();
    }
};

int main() {
    PID pid_vx(0.1, 12, 0.0028, 0.1);      
    PID pid_omega(0.1, 12, 0.0028, 0.1);
    
    Robot robot(0.1);
    double dt = 0.1;
    
    double target_Vx = 1;     
    double target_Omega = 0.5; 
    
    double previous_v_left = 0;
    double previous_omega_left = 0;
    double previous_v_right = 0;
    double previous_omega_right = 0;

    ofstream file("PIDvo.csv");
        file << "Time," << "Vx,Omega_z," << "Target_Vx,Target_Omega," << "Error_Vx,Error_Omega,"
             << "V_left,V_right," << "Omega_left,Omega_right,\n";


    for(int i = 0; i < 1000; i++) {
        
        double current_Vx, current_Omega;
        robot.getVelocities(current_Vx, current_Omega);
        
        double error_Vx    = target_Vx - current_Vx;
        double error_Omega = target_Omega - current_Omega;
        
        double Vx    = pid_vx.calculate(target_Vx, current_Vx);
        double omega = pid_omega.calculate(target_Omega, current_Omega);
        
        robot.apply(Vx, omega);
        
        double V_left, V_right, Omega_left, Omega_right;
        robot.getMotorData(V_left, V_right, Omega_left, Omega_right);

        double time = i * dt;
        file << time << ","
             << current_Vx << ","
             << current_Omega << ","
             << target_Vx << ","
             << target_Omega << ","
             << error_Vx << ","
             << error_Omega << ","
             << V_left << ","
             << V_right << ","
             << Omega_left << ","
             << Omega_right << "\n";
        
        if (i % 10 == 0) {
            cout << "Time " << time << "s"
                 << ": Vx=" << current_Vx 
                 << ", Omega=" << current_Omega
                 << " | Left=" << V_left
                 << ", Right=" << V_right << endl;
        }

        double acc_left    = (V_left - previous_v_left) / dt;
        double alpha_left  = (Omega_left - previous_omega_left) / dt;

        double acc_right   = (V_right - previous_v_right) / dt;
        double alpha_right = (Omega_right - previous_omega_right) / dt;

        if (acc_left    >= 0.75) { V_left      = (0.5 * dt) + previous_v_left; }
        if (acc_right   >= 0.75) { V_right     = (0.5 * dt) + previous_v_right; }
        if (alpha_left  >= 0.25) { Omega_left  = (0.5 * dt) + previous_omega_left; }
        if (alpha_right >= 0.25) { Omega_right = (0.5 * dt) + previous_omega_right; }

        previous_v_left = V_left;
        previous_omega_left = Omega_left;
        previous_v_right = V_right;
        previous_omega_right = Omega_right;

    }
    
    file.close();
    cout << "\n Data saved" << endl;
    return 0;
}