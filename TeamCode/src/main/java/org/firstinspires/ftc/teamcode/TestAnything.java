package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    public Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        grabber = hardwareMap.servo.get("grabber");
        waitForStart();
        while(!isStopRequested()){
            grabber.setPosition(0);
            telemetry.addLine("lol");
            telemetry.update();
            sleep(1000);
            grabber.setPosition(1);
        }
    }
}
