/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name="AutoScore Element -- TunToPosition", group="Pushbot")
//@Disabled
public class ChrisAutoRunToPositionTest extends LinearOpMode {

    /* Declare OpMode members. */
    TechiesHardware robot = new TechiesHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        //initiateRobot();

        robot.testPIDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.testPIDmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED, -10,  5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed,
                             double inches,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = robot.testPIDmotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            robot.testPIDmotor.setTargetPosition(newTarget);
            // Turn On RUN_TO_POSITION
            robot.testPIDmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.testPIDmotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.testPIDmotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newTarget);

                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.testPIDmotor.getCurrentPosition()
                        );
                telemetry.update();
            }

            // Stop all motion;
            robot.testPIDmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.testPIDmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }


    protected void initiateRobot() {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.testPIDmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.testPIDmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.testPIDmotor.getCurrentPosition(),
                robot.testPIDmotor.getCurrentPosition());
        telemetry.update();
    }




}
