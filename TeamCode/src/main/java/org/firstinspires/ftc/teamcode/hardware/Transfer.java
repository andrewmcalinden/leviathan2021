package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public DcMotorEx transfer;
    public Servo pusher;

    public final static double MAX_VELOCITY = 0;

    public Transfer(LinearOpMode opMode){
        transfer = opMode.hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setVelocityPIDFCoefficients(0, 0, 0, 0);
        spinUp();

        pusher = opMode.hardwareMap.get(Servo.class, "pusher");
        retract();
    }

    public Transfer(OpMode opMode){
        transfer = opMode.hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setVelocityPIDFCoefficients(0, 0, 0, 0);
        spinUp();

        pusher = opMode.hardwareMap.get(Servo.class, "pusher");
        retract();
    }

    public void push(){
        pusher.setPosition(1);
    }

    public void retract(){
        pusher.setPosition(0);
    }

    public void spinUp(){
        transfer.setVelocity(MAX_VELOCITY);
    }

    public void stopSpinning(){
        transfer.setVelocity(0);
    }
}
