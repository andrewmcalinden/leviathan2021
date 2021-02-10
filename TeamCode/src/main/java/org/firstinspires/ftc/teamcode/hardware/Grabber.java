package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Grabber {
    public DcMotor arm;
    public Servo grabber;

    public Grabber(LinearOpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection((DcMotor.Direction.FORWARD));

        grabber = opMode.hardwareMap.servo.get("grabber");
        closeGrabber();
    }

    public Grabber(OpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection((DcMotor.Direction.FORWARD));

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
    }

    public void update(double power, boolean close, boolean open){
        if (power == 0) arm.setPower(0);
        else
        {
            arm.setPower(-.1 + Math.abs(power) * power * .6);
        }
        if(close){
            closeGrabber();
        }
        if(open){
            openGrabber();
        }
    }

    public void closeGrabber(){
        grabber.setPosition(.2);
    }

    public void openGrabber(){
        grabber.setPosition(.4);
    }
}
