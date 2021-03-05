package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);

        telemetry.addData("final heading", GlobalVars.finalHeading);
        telemetry.update();
        sleep(2000);

        waitForStart();
        GlobalVars.finalHeading = dt.gyro.getAngle();
        telemetry.addData("final heading", GlobalVars.finalHeading);
        telemetry.update();
    }
}

//make goToPos method for grabber
//fix spaghet
//add pid to strafe gyro
