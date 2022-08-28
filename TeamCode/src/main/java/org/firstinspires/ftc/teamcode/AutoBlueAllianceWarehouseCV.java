/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BlueAllianceWarehouseCV", group = "Concept")
//@Disabled
public class AutoBlueAllianceWarehouseCV extends LinearOpMode {
    TechiesPipeline pipeline;
    protected static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    protected static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    int targetLevel = Constants.TARGET_LEVEL_DEFAULT;


    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot ;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;
    //SlideMovementPIDController pidController;


    @Override
    public void runOpMode() {
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        pipeline= new TechiesPipeline();
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);
        pipeline.setupCamera(hardwareMap);
        targetLevel = determineTargetLevel();
        telemetry.addData("Target Level", targetLevel);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        doMissions(targetLevel);
    }



    private int determineTargetLevel() {
        while (!opModeIsActive())
        {
            telemetry.addData("Freight Location: ", pipeline.getPosition());
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(40);
        }
        if (TechiesPipeline.FreightLocation.ONE.equals(pipeline.getPosition())){
            targetLevel =Constants.TARGET_LEVEL_BOTTOM;
        }
        else if (TechiesPipeline.FreightLocation.TWO.equals(pipeline.getPosition())){
            targetLevel =Constants.TARGET_LEVEL_MIDDLE;
        }
        else if (TechiesPipeline.FreightLocation.THREE.equals(pipeline.getPosition())){
            targetLevel =Constants.TARGET_LEVEL_TOP;
        }
        return targetLevel;
    }

    protected void dropPreloadFreight()   {
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");
        telemetry.update();

        if (Constants.TARGET_LEVEL_BOTTOM == targetLevel) {
            robot.setBucketPower(-.3,.3);
            sleep(400);
            robot.setBucketPower(0,0);
            SlideMovementPID(100);
            robot.horizontalSlide.setPosition(.60);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(800);
            robot.setBucketPower(.25,-.25);
            sleep(800);
            robot.horizontalSlide.setPosition(.3);
            robot.slides.retractSlides();


        } else if (Constants.TARGET_LEVEL_MIDDLE == targetLevel) {
            robot.setBucketPower(-.3,.3);
            sleep(400);
            robot.setBucketPower(0,0);
            SlideMovementPID(275);
            robot.horizontalSlide.setPosition(.60);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(1000);
            robot.setBucketPower(.2,-.2);
            sleep(150);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.retractSlides();

        } else {
            robot.setBucketPower(-.4,.4);
            sleep(500);
            robot.setBucketPower(0,0);
            SlideMovementPID(470);
            robot.horizontalSlide.setPosition(.60);
            sleep(1000);
            robot.setBucketPower(-.25,.25);
            sleep(650);
            robot.setBucketPower(.3,-.3);
            sleep(175);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.retractSlides();
        }


    }
    protected void SlideMovementPID (int targetPosition) {
        robot.slides.rightriser.setDirection(DcMotorEx.Direction.FORWARD);
        robot.slides.leftriser.setDirection(DcMotorEx.Direction.FORWARD);
        telemetry.addData("SlideMovementPID", "start SlideMovementPID");
        robot.slides.rightriser.setTargetPosition(targetPosition);
        robot.slides.leftriser.setTargetPosition(-targetPosition);
        robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (robot.slides.rightriser.isBusy() && repetitions < 800) {

            robot.slides.rightriser.setPower(0.5);
            robot.slides.leftriser.setPower(0.5);

        }
        else {
            robot.slides.rightriser.setPower(0);
            robot.slides.leftriser.setPower(0);
            repetitions = 0;
        }
        currentVelocity = robot.slides.rightriser.getVelocity();
        currentPos = robot.slides.leftriser.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- robot.slides.rightriser.getTargetPosition());
        telemetry.addData("power", robot.slides.rightriser.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }


    protected void doMissions(int targetZone) {

        goToAllianceHubFromStart();
        dropPreloadFreight();
        goToWarehouseFromAllianceHub();
        intake();
        fromWarehouseToHub();
        dropFreightTopLevel();
        park();
        //swivel();
    }

    protected void goToAllianceHubFromStart(){
        /* this is for going straight
        Pose2d startPose = new Pose2d(-48,-25, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartWarehouseBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-24, -50.5, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartWarehouseBlue);*/
        // this is for going diagonally
        Pose2d startPose = new Pose2d(-48,-25, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartWarehouseBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-23, -39, Math.toRadians(135)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartWarehouseBlue);
    }


    protected void goToWarehouseFromAllianceHub() {
        //for straight
        //Pose2d endPoseAllianceHub = new Pose2d(-24,-50.5, Math.toRadians(180));
        Pose2d endPoseAllianceHub = new Pose2d(-23,-39, Math.toRadians(135));
        Trajectory goToWarehouseFromAllianceHub = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                //.lineToLinearHeading(new Pose2d(-56, -30, Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(-60, -30, Math.toRadians(95)))
                .build();
        Trajectory goToWarehouseFromAllianceHub2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-56,-30,Math.toRadians(95)))
                .forward(50)
                .build();
        //TODO: need change so it doens't hit bar
        odoDriveTrain.followTrajectory(goToWarehouseFromAllianceHub);
        odoDriveTrain.followTrajectory(goToWarehouseFromAllianceHub2);


    }
    protected void fromWarehouseToHub() {
        Trajectory fromWarehouseToHub1 = odoDriveTrain.trajectoryBuilder(new Pose2d(-56,30,Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(-56, -15, Math.toRadians(85)))
                .build();
        Trajectory fromWarehouseToHub2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-56,-15,Math.toRadians(85)))
                .lineToLinearHeading(new Pose2d(-29, -30, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(fromWarehouseToHub1);
        odoDriveTrain.followTrajectory(fromWarehouseToHub2);
    }
    protected void dropFreightTopLevel() {
        robot.setBucketPower(-.4,.4);
        sleep(500);
        robot.setBucketPower(0,0);
        SlideMovementPID(470);
        robot.horizontalSlide.setPosition(.75);
        sleep(1000);
        robot.setBucketPower(-.3,.3);
        sleep(800);
        robot.setBucketPower(.2,-.2);
        sleep(200);
        robot.horizontalSlide.setPosition(.32);
        sleep(100);
        robot.slides.retractSlides();
    }



    protected void park() {
        Trajectory parkWarehouseBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(-29,-30,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-65, 0, Math.toRadians(95)))
                .build();
        Trajectory parkWarehouseBlue2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-65,0,Math.toRadians(95)))
                .forward(45)
                .build();
        odoDriveTrain.followTrajectory(parkWarehouseBlue);
        odoDriveTrain.followTrajectory(parkWarehouseBlue2);
    }
    protected void swivel() {
        Pose2d endPoseAllianceHub = new Pose2d(-55,41, Math.toRadians(95));
        Trajectory parkInWarehouseFromAllianceHub = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .strafeRight(22)
                .build();
        Trajectory parkInWarehouseFromAllianceHub2 = odoDriveTrain.trajectoryBuilder(parkInWarehouseFromAllianceHub.end())
                .lineToLinearHeading(new Pose2d(-37, 75, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(parkInWarehouseFromAllianceHub);
        odoDriveTrain.followTrajectory(parkInWarehouseFromAllianceHub2);
    }

    protected void intake() {
        robot.setBucketPower(0,0);
        robot.intake.setPower(-1);
        sleep(2000);
        robot.intake.setPower(1);
        sleep(500);
        robot.intake.setPower(0);
    }



}

