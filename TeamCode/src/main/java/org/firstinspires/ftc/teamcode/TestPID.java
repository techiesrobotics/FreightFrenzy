package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TestPID", group="Linear Opmode")
// right now using only one motor
public class TestPID extends LinearOpMode {

        // motor declaration, we use the
        // Ex version as it has velocity measurements
        DcMotorEx motor;
        // create our PID controller, you will need to tune these parameters
        TechiesPIDController control = new TechiesPIDController(0.05,0,0);
    PIDFController rrController = new PIDFController(new PIDCoefficients(0,0,0));
        @Override
        public void runOpMode() throws InterruptedException {
            // the string is the hardware map name
            motor = hardwareMap.get(DcMotorEx.class, "arm"); //TODO use the real one

            // use braking to slow the motor down faster
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // disables the default velocity control
            // this does NOT disable the encoder from counting,
            // but lets us simply send raw motor power.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            waitForStart();
            // loop that runs while the program should run.
            // position in encoder ticks where we would like the motor to run
            int targetPosition = 100;

            while (opModeIsActive()) {
                // update pid controller
                double command = control.update(targetPosition,
                        motor.getCurrentPosition());
                // assign motor the PID output
                motor.setPower(command);
            }
        }

}
