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

package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechiesDriveTrainHardware;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;

@TeleOp(name="Techies OpMode 2 player", group="Linear Opmode")
public class TechiesOpModeMecanum2Player extends LinearOpMode {

    // Declare OpMode members.
    TechiesDriveTrainHardware robot   = new TechiesDriveTrainHardware();
    private ElapsedTime runtime = new ElapsedTime();
    TechiesHardwareWithoutDriveTrain robotCore ;

    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;


    @Override
    public void runOpMode() {
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double backleftPower;
            double backrightPower;

            double turn = gamepad2.right_stick_x;
            double drivefb  = -gamepad2.left_stick_y;
            double drivelr = gamepad2.left_stick_x;

            leftPower    = Range.clip(drivefb + turn + drivelr, -1.0, 1.0) ;
            rightPower   = Range.clip(drivefb - turn - drivelr, -1.0, 1.0) ;
            backleftPower   = Range.clip(drivefb + turn - drivelr, -1.0, 1.0) ;
            backrightPower   = Range.clip(drivefb - turn + drivelr, -1.0, 1.0) ;

            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.leftBack.setPower(backleftPower);
            robot.rightBack.setPower(backrightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            if (gamepad1.right_bumper) {
                robotCore.intake.setPower(-1);
            }
            else {
                robotCore.intake.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robotCore.intake.setPower(1);
            }
            else {
                robotCore.intake.setPower(0);
            }

            if (gamepad1.a) {
                SlideMovementPID(20);
            }

            if (gamepad1.b) {
                SlideMovementPID(150);
            }
            if (gamepad1.y) {
                SlideMovementPID(400);
            }
            if (gamepad1.x) {
                robotCore.slides.retractSlides();
            }

            if (gamepad1.dpad_up) {
                robotCore.setBucketPower(.45, -.45);
            } else {
                robotCore.setBucketPower(0,0);
            }

            if (gamepad1.dpad_down) {
                robotCore.setBucketPower(-.45, .45);
            } else {
                robotCore.setBucketPower(0,0);
            }

            if (gamepad1.dpad_right) {
                robotCore.duckMech.setPosition(1);
                sleep(2500);
                robotCore.duckMech.setPosition(.5);
            }
            if (gamepad1.dpad_left) {
                robotCore.duckMech.setPosition(-1);
                sleep(2500);
                robotCore.duckMech.setPosition(.5);
            }
            if (gamepad2.dpad_up) {
                robotCore.horizontalSlide.setPosition(.8);
            }

            if (gamepad2.dpad_down) {
                robotCore.horizontalSlide.setPosition(.3);
            }

        }
    }

    protected void SlideMovementPID (int targetPosition) {
        telemetry.addData("SlideMovementPID", "start SlideMovementPID");
        robotCore.slides.setTargetPosition(targetPosition,-targetPosition);
        robotCore.slides.rightriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotCore.slides.leftriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (robotCore.slides.rightriser.isBusy() && repetitions < 800) {
            robotCore.slides.setRiserPower(0.5,0.5);
        }
        else {
            robotCore.slides.setRiserPower(0,0);
            repetitions = 0;
        }
        currentVelocity = robotCore.slides.rightriser.getVelocity();
        currentPos = robotCore.slides.leftriser.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- robotCore.slides.rightriser.getTargetPosition());
        telemetry.addData("power", robotCore.slides.rightriser.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }
}
