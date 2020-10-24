package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    public Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        grabber = hardwareMap.servo.get("grabber");
        waitForStart();
        grabber.setPosition(.5);
        telemetry.addLine("lol");
        telemetry.update();
        sleep(2000);
        telemetry.update();
        grabber.setPosition(.7);
    }
}
