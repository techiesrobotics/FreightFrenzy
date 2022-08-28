package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionHelper {
    Telemetry telemetry;
    private OpenCvWebcam webcam;
    private TechiesPipeline pipeline;

    public VisionHelper( HardwareMap hardwareMap, String webcamName, Telemetry telemetry ) {
        this.telemetry = telemetry;
      //  setup( hardwareMap, webcamName );
    }

    public void setUpCamera(HardwareMap hardwareMap, TechiesPipeline aPipeline  )
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        System.out.println("inside setup camera");
        pipeline = aPipeline; //new TechiesPipeline();
        webcam.setPipeline(pipeline);
        System.out.println("inside setup camera, setup pipeline");

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
                System.out.println("opened");
            }

            @Override
            public void onError(int errorCode)
            {
                System.out.println("error, camera could not be opened");
                // This will be called if the camera could not be opened

            }
        });
    }
    /*
    public void setup( HardwareMap hardwareMap, String webcamName ) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) );
        webcam = OpenCvCameraFactory.getInstance( ).createWebcam( hardwareMap.get( WebcamName.class, webcamName ), cameraMonitorViewId );
        pipeline = new TechiesPipeline(  ); //TODO need to add log?
        webcam.setPipeline( pipeline );

    }

    public void init( ) {
        openCameraDeviceKL( );
    }

    public void setTimeoutTime( int milliseconds ) {
        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout( milliseconds );
    }

    public void openCameraDeviceKL() {

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
/*
    public void openCameraDevice( ) {

        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened( ) {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }

            @Override
            public void onError( int errorCode ) {
                //This will be called if the camera could not be opened
               // Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
            }
        } );
    }
*/
    /*
    public Constants.FreightLocation getFreightLocation( ) {
        return pipeline.getPosition( );
    }

    public void stopCamera( ) {
        webcam.stopStreaming( );
    }*/
}

