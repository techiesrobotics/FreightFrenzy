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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
@Autonomous(name = "KLAutoTestCV", group = "Concept")
//@Disabled
public class KLAutoTestCV extends LinearOpMode {
    OpenCvCamera webcam;
    TechiesPipeline pipeline;
    protected static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    TechiesPipeline.FreightLocation targetLevel = TechiesPipeline.FreightLocation.NOTFOUND;


    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot ;
    VisionHelper visionHelper;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;
    //SlideMovementPIDController pidController;


    @Override
    public void runOpMode() {
        visionHelper = new VisionHelper(hardwareMap, "Webcam 1",telemetry);
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);

      //  pipeline = new TechiesPipeline();
     //   visionHelper.setUpCamera(hardwareMap, pipeline);

        setupCamera();
        targetLevel = determineTargetLevel();
        telemetry.addData("Target Level", targetLevel);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

       // doMissions(targetLevel);
    }

    private void setupCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TechiesPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                System.out.println("onOpened, camera opened");
                webcam.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                System.out.println("error, camera could not be opened");
                 // This will be called if the camera could not be opened

            }
        });
    }

    private TechiesPipeline.FreightLocation determineTargetLevel() {
        while (!opModeIsActive())
        {
            telemetry.addData("Freight target level: ", pipeline.getTargetLevel());
           // telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(40);
        }
        targetLevel = pipeline.getPosition();
        return targetLevel;
    }

    protected void dropPreloadFreight()   {
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");
        telemetry.update();

        if (Constants.TARGET_LEVEL_BOTTOM == pipeline.getTargetLevel()) {
            robot.setBucketPower(-.3,.3);
            sleep(400);
            robot.setBucketPower(0,0);
            SlideMovementPID(100);
            robot.horizontalSlide.setPosition(.60);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(800);
            robot.setBucketPower(.25,-.25);
            sleep(600);
            robot.horizontalSlide.setPosition(.3);
            robot.slides.retractSlides();


        } else if (Constants.TARGET_LEVEL_MIDDLE == pipeline.getTargetLevel()) {
            robot.setBucketPower(-.2,.2);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(275);
            robot.horizontalSlide.setPosition(.75);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(1000);
            robot.setBucketPower(.2,-.2);
            sleep(150);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.retractSlides();

        } else {
            robot.setBucketPower(-.3,.3);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(470);
            robot.horizontalSlide.setPosition(.75);
            sleep(1000);
            robot.setBucketPower(-.3,.3);
            sleep(700);
            robot.setBucketPower(.1,-.1);
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
        goToCarousel();
        spinCarousel();
        parkInBlueStorage();
    }

    protected void goToAllianceHubFromStart(){
        /* for straight
        Pose2d startPose = new Pose2d(-48,-75, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -50.5, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);
        */
        Pose2d startPose = new Pose2d(-48,-75, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-24, -61, Math.toRadians(225)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);
    }

    protected void goToCarousel() {
        Pose2d endPoseAllianceHub = new Pose2d(-24,-61, Math.toRadians(225));
        Trajectory goToCarouselDuckBlue = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .lineToLinearHeading(new Pose2d(-50, -138, Math.toRadians(185)))
                .build();
        odoDriveTrain.followTrajectory(goToCarouselDuckBlue);
    }

    protected void spinCarousel() {
        robot.setBucketPower(0,0);
        robot.duckMech.setPosition(1);
        sleep(6000);
        robot.duckMech.setPosition(.5);
    }

    protected void parkInBlueStorage(){
        Trajectory parkBlueStorage = odoDriveTrain.trajectoryBuilder(new Pose2d(-52,-130,Math.toRadians(185)))
                .lineToLinearHeading(new Pose2d(-23, -130, Math.toRadians(90)))
                .build();
        odoDriveTrain.followTrajectory(parkBlueStorage);
    }

    protected void park() {
        Trajectory parkDuckBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(-50,-130,Math.toRadians(185)))
                .lineToLinearHeading(new Pose2d(-65, -100, Math.toRadians(75)))
                .build();
        Trajectory parkDuckBlue2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-65,-30,Math.toRadians(100)))
                .forward(107)
                .build();
        odoDriveTrain.followTrajectory(parkDuckBlue);
        odoDriveTrain.followTrajectory(parkDuckBlue2);
    }


}

