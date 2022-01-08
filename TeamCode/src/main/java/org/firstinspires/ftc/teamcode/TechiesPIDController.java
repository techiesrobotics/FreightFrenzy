package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TechiesPIDController {
    /**
     * construct PID controller
     */
    // param Kp Proportional coefficient
    // @param Ki Integral coefficient
    // @param Kd Derivative coefficient
    public TechiesPIDController(double aKp, double aKi, double aKd) {
        double Kp = aKp;
        double Ki = aKi;
        double Kd = aKd;


    }

    /**
     * update the PID controller output
     *
     * @param target where we would like to be, also called the reference
     * @param state  where we currently are, I.E. motor position
     *               return the command to our motor, I.E. motor power
     */
    //public double update(double target, double state) {
    public double update(double target, double state) {
        // PID logic and then return the output
        double reference = 0; // TODO figure out what value to put

        double integralSum = 0;

        double lastError = 0;

        boolean setPointIsNotReached = false;
// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
        // obtain the encoder position
        double encoderPosition = state; //TODO armMotor.getPosition();

        while (setPointIsNotReached) {

            // calculate the error
            double error = reference - encoderPosition;


            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = 0; //(Kp * error) + (Ki * integralSum) + (Kd * derivative);
/*
            armMotor.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();


            return 0.5; // TODO need to find the real return value
            */

        }
        return 0.5
                ;
    }
}


