package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Grabber {
    public DcMotor arm;
    public Servo grabber;

    public OpMode myOpmode;

    private double startPos;
    private boolean lastButtonPressed;
    private boolean open;

    private boolean lastMacro;
    private boolean up;

    public Grabber(LinearOpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setDirection((DcMotor.Direction.FORWARD));

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        lastMacro = false;
        up = false;
    }

    public Grabber(OpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection((DcMotor.Direction.FORWARD));
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        lastMacro = false;
        up = false;

        myOpmode = opMode;
    }
    //this is some cheese, should probably comment out
    public void update(double power, boolean buttonPressed, boolean macro){
        if (power == 0){
            arm.setPower(0);
            if (macro && !lastMacro){
                if (up){
                    myOpmode.telemetry.addLine("going down");
                    myOpmode.telemetry.update();
                    putDown();
                    up = false;
                }
                else{
                    liftUp();
                    myOpmode.telemetry.addLine("going up");
                    myOpmode.telemetry.update();
                    up = true;
                }
            }
            lastMacro = macro;
        }
        else{
            arm.setPower(-.1 + Math.abs(power) * power * .6);
            myOpmode.telemetry.addData("power", -.1 + Math.abs(power) * power * .6);
        }
        if(buttonPressed && !lastButtonPressed){
            if (open){
                closeGrabber();
                open = false;
            }
            else{
                openGrabber();
                open = true;
            }
        }
        lastButtonPressed = buttonPressed;
        myOpmode.telemetry.addData("position", arm.getCurrentPosition());
        myOpmode.telemetry.update();
    }

    public void liftUp(){
        while(arm.getCurrentPosition() < startPos + 180){
            arm.setPower(-.3);
        }
        arm.setPower(0);
    }

    public void putDown(){
        while(arm.getCurrentPosition() > startPos){
            arm.setPower(.1);
        }
        arm.setPower(0);
    }

    public void closeGrabber(){
        grabber.setPosition(.25);
    }
    public void openGrabber(){
        grabber.setPosition(.4);
    }
}
