package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestChris",group="Linear Opmode Test")
public class XTestPIDChrisAutoChris extends XTestPIDChrisAuto {

    void testChildMethod() {

        telemetry.addData("===============it should  show up", "from Christopher");
        telemetry.update();
    }
}