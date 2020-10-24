package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {
    public DcMotorEx shooter;

    public final static double MAX_VELOCITY = 0;

    public Shooter(LinearOpMode opMode){
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(0, 0, 0, 0);
        startShooting();
    }

    public Shooter(OpMode opMode){
        shooter = opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(0, 0, 0, 0);
        startShooting();
    }

    public void startShooting(){
        shooter.setVelocity(MAX_VELOCITY);
    }

    public void stopShooting(){
        shooter.setVelocity(0);
    }

}
