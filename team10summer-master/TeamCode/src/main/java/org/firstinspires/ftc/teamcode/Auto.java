package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
public class Auto extends LinearOpMode {
    Drivetrain drive;
    Arm servo;
    Vision eye;


    @Override
    public void runOpMode() throws InterruptedException {
        drive.initv2(this);
        servo.initv2(this);
        eye.initv2(this);
        int skyStone = eye.skyStonePos();

        waitForStart();

        drive.movePIDFGyro(2,1,1,1,1);
        drive.trunPIDF(45,1,1,1,1);
        drive.movePIDFGyro(14,1,1,1,1);
        servo.dropProduct();
        sleep(1000);
        servo.reset();
        drive.movePIDFGyro(-2,1,1,1,1);
        drive.trunPIDF(-95,1,1,1,1);
        drive.movePIDFGyro(8,1,1,1,1); //YEEET



    }



}
