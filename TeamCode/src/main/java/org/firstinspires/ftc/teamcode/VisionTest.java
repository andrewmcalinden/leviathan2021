package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VisionTest", group = "18030")
public class VisionTest extends LinearOpMode {
    public Vision ringCounter;

    @Override
    public void runOpMode() throws InterruptedException {
        ringCounter = new Vision(this);
        while(!isStarted()){
            int num = ringCounter.numRingsLeftSide();
            telemetry.addData("num rings: ", num);
            telemetry.update();
        }
        waitForStart();
    }
}
