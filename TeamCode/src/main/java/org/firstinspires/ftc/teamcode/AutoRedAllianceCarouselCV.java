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
import org.firstinspires.ftc.teamcode.test.SlideMovementPIDController;
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
@Autonomous(name = "RedAllianceCarouselCV", group = "Concept")
//@Disabled
public class AutoRedAllianceCarouselCV extends LinearOpMode {
    OpenCvCamera webcam;
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
    SlideMovementPIDController pidController;


    @Override
    public void runOpMode() {
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);

        setupCamera();

        targetLevel = determineTargetLevel();
        telemetry.addData("Target Level", targetLevel);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        doRedCarouselMissions(targetLevel);
    }


    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TechiesPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

                 // This will be called if the camera could not be opened

            }
        });
    }



     private int determineTargetLevel() {
        while (!opModeIsActive())
        {
            telemetry.addData("Freight Location: ", pipeline.getPosition());
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(40);
        }
         if (TechiesPipeline.FreightLocation.TWO.equals(pipeline.getPosition())) {
             targetLevel = Constants.TARGET_LEVEL_MIDDLE;
         }
         else if (TechiesPipeline.FreightLocation.ONE.equals(pipeline.getPosition())){
            targetLevel =Constants.TARGET_LEVEL_BOTTOM;
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
            robot.setBucketPower(-3,.3);
            sleep(450);
            robot.setBucketPower(0,0);
            SlideMovementPID(100);
            robot.horizontalSlide.setPosition(.65);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(800);
            robot.setBucketPower(.3,-.3);
            sleep(650);
            robot.horizontalSlide.setPosition(.3);
            robot.slides.retractSlides();


        } else if (Constants.TARGET_LEVEL_MIDDLE == targetLevel) {
            robot.setBucketPower(-.2,.2);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(275);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(1000);
            robot.setBucketPower(.2,-.2);
            sleep(150);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.retractSlides();

        } else {
            robot.setBucketPower(-.35,.35);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(470);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.setBucketPower(-.3,.3);
            sleep(700);
            robot.setBucketPower(.25,-.25);
            sleep(500);
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


    protected void doRedCarouselMissions(int targetZone) {

        goToRedAllianceHubFromStart();
        dropPreloadFreight();
        goToRedCarousel();
        spinRedCarousel();
        //parkInRedWarehouse();
        parkInRedStorage();
    }

    protected void goToRedAllianceHubFromStart(){
        //for straight
        /*Pose2d startPose = new Pose2d(48,-75, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(26, -50, Math.toRadians(0)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);*/
        Pose2d startPose = new Pose2d(48,-75, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckRed = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, -61, Math.toRadians(-45)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckRed);
    }

    protected void goToRedCarousel() {
        Pose2d endPoseAllianceHub = new Pose2d(24,-61, Math.toRadians(-45));
        Trajectory goToCarouselDuckBlue = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .lineToLinearHeading(new Pose2d(58, -130, Math.toRadians(0)))
                .build();
        odoDriveTrain.followTrajectory(goToCarouselDuckBlue);
    }

    protected void spinRedCarousel() {
        // Change the direction of the servo - AJ 2/5
        robot.setBucketPower(0,0);
        robot.duckMech.setPosition(0);
        sleep(6000);
        robot.duckMech.setPosition(.5);
    }
    protected void parkInRedStorage(){
        Trajectory parkRedStorage = odoDriveTrain.trajectoryBuilder(new Pose2d(58,-130,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(28, -130, Math.toRadians(90)))
                .build();
        odoDriveTrain.followTrajectory(parkRedStorage);
    }


    protected void parkInRedWarehouse() {
       // sleep(10000);
        Trajectory parkDuckBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(56,-130,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(65, -100, Math.toRadians(83)))
                .build();
        Trajectory parkDuckBlue2 = odoDriveTrain.trajectoryBuilder(new Pose2d(65,-30,Math.toRadians(83)))
                .forward(107)
                .build();
        odoDriveTrain.followTrajectory(parkDuckBlue);
        odoDriveTrain.followTrajectory(parkDuckBlue2);
    }


}

