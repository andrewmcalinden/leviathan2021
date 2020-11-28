package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo servo;
    public Servo gate;

    public void initv2(LinearOpMode opMode) {
        gate = opMode.hardwareMap.get(Servo.class, "gate");
        servo = opMode.hardwareMap.get(Servo.class, "servo");
        servo.setPosition(.5);

    }
    public void dropProduct(){
        servo.setPosition(.8);
    }

    public void reset(){
        servo.setPosition(.5);
    }

    public void openGate(){
        gate.setPosition(.123);
    }

    public void closeGate(){
        gate.setPosition(.9999);
    }

}
