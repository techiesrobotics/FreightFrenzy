package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestNathan",group="Linear Opmode Test")
public class XTestPIDChrisAutoNathan extends XTestPIDChrisAuto {

    void testChildMethod() {

        telemetry.addData("===============it should  show up", "from Nathan");
        telemetry.update();
    }
}