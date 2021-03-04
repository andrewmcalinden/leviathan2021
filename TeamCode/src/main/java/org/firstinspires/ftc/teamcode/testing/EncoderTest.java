package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Disabled
public class EncoderTest extends LinearOpMode {
    Drivetrain dt;


    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("fl", dt.fL.getCurrentPosition());
            telemetry.addData("fr", dt.fR.getCurrentPosition());
            telemetry.addData("bl", dt.bL.getCurrentPosition());
            telemetry.addData("br", dt.bR.getCurrentPosition());
            telemetry.update();
        }
    }
}
