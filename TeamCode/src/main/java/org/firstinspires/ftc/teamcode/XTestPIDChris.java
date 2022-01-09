package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ChrisPIDPositionTest",group="Linear Opmode Test")
public class XTestPIDChris extends LinearOpMode {
    DcMotorEx motor;
    DcMotorEx motor2;

    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    //double maxPos = 0.0;
    double repetitions = 0;
    //double integral = 0.0;
    //ElapsedTime PIDtimer=  new ElapsedTime();
    //PIDCoefficients testPIDCofficients = new PIDCoefficients(1.2,0.220,0);

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "PID");
        motor2 = hardwareMap.get(DcMotorEx.class, "PID2");

        setUpMotor(motor);
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        setUpMotor(motor2);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        waitForStart();
        // Pre-run
        while (opModeIsActive()) {

           // if (gamepad1.right_bumper){
                moveTestMotorDiscord();
            //}

         //   if(gamepad1.left_bumper) {
           //    moveTestMotorYouTubeTutorial(1300);
            //}

        }
    }

    private void setUpMotor(DcMotorEx aMotor) {

        aMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setVelocityPIDFCoefficients(1.20,.220, 0,10.996); //Change these
        aMotor.setPositionPIDFCoefficients(5.0);
        aMotor.setTargetPositionTolerance(50); //Maybe change this


    }


    private void moveTestMotorDiscord() {
        // if(gamepad1.) motor.setTargetPosition(0); // Change these
        motor2.setTargetPosition(-550);
        motor.setTargetPosition(550);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motor.isBusy() && repetitions < 800) {
            motor.setPower(0.6);
            motor2.setPower(0.6);

        }
        else{
            motor.setPower(0);
            motor2.setPower(0);
            repetitions = 0;
        }
        currentVelocity = motor.getVelocity();
        currentPos = motor.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos-motor.getTargetPosition());
        telemetry.addData("power", motor.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }
/*
    public void moveTestMotorYouTubeTutorial(double targetPosition)
    {
        double error = motor.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9){
            telemetry.addData("error", error);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();

            error = motor.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
             integral += changeInError * PIDtimer.time();
             double derivative = changeInError / PIDtimer.time();;
             double P = testPIDCofficients.p * error;
             double I = testPIDCofficients.i * integral;
             double D = testPIDCofficients.d * derivative;
             motor.setPower(0.5 + P + I + D);
            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("Derivative", derivative);
            telemetry.update();
            error = lastError;

            PIDtimer.reset();


        }
    }*/
}