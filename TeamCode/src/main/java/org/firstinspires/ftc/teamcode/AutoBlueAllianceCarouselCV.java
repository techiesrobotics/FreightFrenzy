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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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
public class AutoBlueAllianceCarouselCV extends LinearOpMode {
    OpenCvCamera webcam;
    TechiesOpenCV pipeline;
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
        // initVuforia();
        // initTfod();
        // activateCamera();

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
    }}
    class TechiesOpenCV extends OpenCvPipeline {
        /*
         * An enum to define the position
         */
        public enum FreightLocation {
            ONE,
            TWO,
            THREE
        }

        /*
         * Some color constants
         */
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar YELLOW = new Scalar(255, 255, 0);
        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 250);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(425, 250);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(840, 250);
        static final int REGION_WIDTH = 70;
        static final int REGION_HEIGHT = 70;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat Y = new Mat();
        Mat Cr = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation position = org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        void inputToY(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Y, 1);
        }

        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 3);
        }

        int maximum = 0;

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the region to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == avg3) // Was it from region 1?
            {
                position = org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg2) // Was it from region 2?
            {
                position = org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.TWO; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg1) // Was it from region 3?
            {
                position = org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE;// Record our analysis

                /*``
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            maximum = max;
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation getAnalysis() {
            return position;
        }

        public int getMaximum() {
            return maximum;
        }
        public int convert (){
            if (getAnalysis().equals(org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE)){
                return 1;
            }
            else if (getAnalysis().equals(org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE)){
                return 2;
            }
            else if (getAnalysis().equals(org.firstinspires.ftc.teamcode.test.FFPipeline.FreightLocation.ONE)){
                return 3;
            }
            else {
                return 3;
            }

        }}


