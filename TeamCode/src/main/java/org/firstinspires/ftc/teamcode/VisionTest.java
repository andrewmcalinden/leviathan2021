package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VisionTest", group = "18030")
public class VisionTest extends LinearOpMode {
    public Vision ringCounter;

    @Override
    public void runOpMode() throws InterruptedException {
        ringCounter = new Vision(this);
        waitForStart();

        while(!isStopRequested()){
            //int numRings = ringCounter.numRings();
            //telemetry.addData("num rings: ", numRings);
            ringCounter.displayColor(495, 365);
        }
    }
}
