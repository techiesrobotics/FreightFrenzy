package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

// @Autonomous(name="ChrisPIDPositionAutoTest",group="Linear Opmode Test")
public abstract class XTestPIDChrisAuto extends LinearOpMode {
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
     ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "PID");
        motor2 = hardwareMap.get(DcMotorEx.class, "PID2");


        waitForStart();

        telemetry.addData("start", "start");
        telemetry.update();
        // Pre-run
        while (opModeIsActive()) {
            testChildMethod();
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

    abstract void testChildMethod() ;
}