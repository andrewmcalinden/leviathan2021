package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VisionTest", group = "18030")
public class VisionTest extends LinearOpMode {
    public Vision ringCounter;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);
        ringCounter = new Vision(this);

        while(!isStarted()){
            int num = ringCounter.numRingsRightSide();
        }
        telemetry.setAutoClear(true);
        telemetry.update();
        waitForStart();
        telemetry.addLine("1");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("2");
        telemetry.update();
        sleep(1000);
    }
}
