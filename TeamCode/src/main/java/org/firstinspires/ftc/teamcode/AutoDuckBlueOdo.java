package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="AutoDuckBlueOdo", group="Pushbot")
//@Disabled
public class AutoDuckBlueOdo extends AutoDuckOdo {


    @Override
    protected void spinCarousel() {
        // TODO add code
    }

    @Override
    protected void goToCarousel() {
        Pose2d endPoseAllianceHub = new Pose2d(-10,-70, Math.toRadians(270));
        Trajectory goToCarouselDuckBlue = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .lineToLinearHeading(new Pose2d(-60, -110, Math.toRadians(270)))
                .build();

        odoDriveTrain.followTrajectory(goToCarouselDuckBlue);
    }

    @Override
    protected void park() {
        Trajectory parkDuckBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(-60,-110,Math.toRadians(270)))
                .strafeLeft(25)
                .build();
        odoDriveTrain.followTrajectory(parkDuckBlue);

    }


    @Override
    protected void goToAllianceHubFromStart(){
        Pose2d startPose = new Pose2d(-48,-74, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10, -70, Math.toRadians(270)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);
    }


}

