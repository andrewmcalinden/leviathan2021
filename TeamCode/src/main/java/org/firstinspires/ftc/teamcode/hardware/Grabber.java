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
        arm.setDirection((DcMotor.Direction.FORWARD));

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        lastMacro = false;
        up = false;

        myOpmode = opMode;
    }

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
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double upPositon = startPos + 180;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition((int) -upPositon);
        myOpmode.telemetry.addData("target:", upPositon);
        arm.setPower(.5);
    }

    public void putDown(){
        double downPosition = startPos;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition((int) downPosition);
        myOpmode.telemetry.addData("target:", downPosition);
        arm.setPower(.5);
    }

    public void closeGrabber(){
        grabber.setPosition(.25);
    }

    public void openGrabber(){
        grabber.setPosition(.4);
    }
}
