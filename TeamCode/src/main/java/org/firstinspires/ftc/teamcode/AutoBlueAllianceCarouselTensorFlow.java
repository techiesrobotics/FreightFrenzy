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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

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
@Autonomous(name = "BlueAllianceCarousel", group = "Concept")
//@Disabled
public class AutoBlueAllianceCarouselTensorFlow extends LinearOpMode {

    protected static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    protected static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    int targetZone = Constants.TARGET_LEVEL_DEFAULT;
    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot ;
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;
    //SlideMovementPIDController pidController;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    protected static final String VUFORIA_KEY =
            " AbfemMf/////AAABmVQ5LwVH3Umfv+Oiv7oNSvcdOBa+ogwEc69mgH/qFNgbn1NXBJsX4J5R6N4SYojjxFB6eHjdTHaT3i9ZymELzgaFrPziL5B/TX2/dkxnIK5dcOjLHOZu2K3jVPYciJnwj20ZmRXSN46Y4uMdzWSJ3X1wgKovNQzZvx+7dljIonRJfLjSF5aSuoTDqEkdcsGTJ92J7jgc5jN53Vml3rAI+qPUTT8qpI8T1enV6NYublcUMpofpovmHsH9kvI+U1h9Rc8cXwbGPlr3PVoKQOwuZA0Y98Jywey6URYTswzpbMw8cWu4PMisB2ujpf0VEjiV6jjofr3OVRj6r5lEJZsnElY2mOhXdgVqJndvXYvCSpyI";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);
        // set up camera
        initVuforia();
        initTfod();
        activateCamera();

        // Wait for the game to begin


       targetZone = determineLevel();
        telemetry.addData(">", "Press Play to start op mode");

        telemetry.update();


        waitForStart();
        telemetry.addData("Target Zone", targetZone);
        telemetry.update();
        telemetry.addData("do something", "do things");

        doMissions(targetZone );
        telemetry.addData("do missions", "finish mission");
        telemetry.update();
    }

    protected int determineLevel() {
        while (!opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());

                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        Double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES); // KL changed
                        telemetry.addData(("  Angle: "), angle);
                        if (recognition.getLabel().equals("Duck")){
                            if (angle.doubleValue() <= -3){
                                targetZone = Constants.TARGET_LEVEL_BOTTOM;
                                telemetry.addData(String.format("  targetZone: "), targetZone);
                            }
                            else if (angle.doubleValue() >= 3){
                                targetZone = Constants.TARGET_LEVEL_MIDDLE;
                                telemetry.addData(String.format("  targetZone: "), targetZone);
                            }
                          }else{
                            telemetry.addData(String.format("  recognition.getLabel(): "), recognition.getLabel());
                        }
                        i++;
                      }
                      telemetry.update();
                    }
                }
            }
        telemetry.addData(("  target zone:========================= "), targetZone);
        return targetZone;
    }

    protected void activateCamera() {
        if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1, 17.0 / 5.0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName =hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.45f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    protected void dropPreloadFreight()   {
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");
        telemetry.update();

        if (Constants.TARGET_LEVEL_BOTTOM == targetZone) {
            telemetry.addData("dosomething for zone 1", "do something for zone 1");
            telemetry.update();

            robot.setBucketPower(-.2,.2);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(100);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(800);
            robot.setBucketPower(.2,-.2);
            sleep(600);
            robot.horizontalSlide.setPosition(.3);
            robot.slides.retractSlides();


        } else if (Constants.TARGET_LEVEL_MIDDLE == targetZone) {
            telemetry.addData("dosomething for zone 2", "do something for zone 2");
            telemetry.update();

            robot.setBucketPower(-.2,.2);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(300);
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
            telemetry.addData("dosomething for zone 3", "do something for zone 3");
            telemetry.update();
            robot.setBucketPower(-.2,.2);
            sleep(350);
            robot.setBucketPower(0,0);
            SlideMovementPID(470);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.setBucketPower(-.2,.2);
            sleep(700);
            robot.setBucketPower(.1,-.1);
            sleep(175);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.retractSlides();
        }


        telemetry.addData("finish", "finish everythimg");
        telemetry.update();


    }
    protected void SlideMovementPID (int targetPosition) {
        telemetry.addData("SlideMovementPID", "start SlideMovementPID");
        robot.slides.rightriser.setTargetPosition(targetPosition);
        robot.slides.leftriser.setTargetPosition(-targetPosition);
        robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(robot.slides.rightriser.isBusy() && repetitions < 800) {

            robot.slides.rightriser.setPower(0.5);
            robot.slides.leftriser.setPower(0.5);
            telemetry.addData("inside if", "inside if");
        }
        else{
            telemetry.addData("SlideMovementPID", "inside  else");
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
       // doAdditionalMissions(targetZone);
        goToCarousel();
        spinCarousel();
        park();
    }

    protected void goToAllianceHubFromStart(){
        Pose2d startPose = new Pose2d(-48,-70, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -50, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);
    }
/*
    protected void goToCarousel() {telemetry.addData("goto carousel from parent", "parent");};
    protected void spinCarousel() {telemetry.addData("spinCarousel from parent", "parent");};
    protected void park() {{telemetry.addData("park from parent", "parent");};};

   */
    protected void goToCarousel() {
        Pose2d endPoseAllianceHub = new Pose2d(-27,-50, Math.toRadians(180));
        Trajectory goToCarouselDuckBlue = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .lineToLinearHeading(new Pose2d(-53, -130, Math.toRadians(185)))
                .build();
        odoDriveTrain.followTrajectory(goToCarouselDuckBlue);
    }

    protected void spinCarousel() {
        robot.setBucketPower(0,0);
        robot.duckMech.setPosition(1);
        sleep(3000);
        robot.duckMech.setPosition(.5);
    }


    protected void park() {
        Trajectory parkDuckBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(-53,-130,Math.toRadians(185)))
                .lineToLinearHeading(new Pose2d(-65, -100, Math.toRadians(75)))
                .build();
        Trajectory parkDuckBlue2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-65,-30,Math.toRadians(100)))
                .forward(107)
                .build();
        odoDriveTrain.followTrajectory(parkDuckBlue);
        odoDriveTrain.followTrajectory(parkDuckBlue2);
    }


}