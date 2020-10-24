package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
        closeGrabber();
    }

    public void update(double power, boolean close, boolean open){
        arm.setPower(Math.abs(power) * power);
        if(close){
            closeGrabber();
        }
        else if(open){
            openGrabber();
        }
    }

    public void closeGrabber(){
        grabber.setPosition(0);
    }

    public void openGrabber(){
        grabber.setPosition(1);
    }
}
