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
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    //this is some cheese, should probably comment out
//    public void update(double power, boolean buttonPressed, boolean macro){
//        if (power == 0){
//            arm.setPower(0);
//            if (macro && !lastMacro){
//                if (up){
//                    myOpmode.telemetry.addLine("going down");
//                    myOpmode.telemetry.update();
//                    putDown();
//                    up = false;
//                }
//                else{
//                    liftUp();
//                    myOpmode.telemetry.addLine("going up");
//                    myOpmode.telemetry.update();
//                    up = true;
//                }
//            }
//            lastMacro = macro;
//        }
//        else{
//            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            arm.setPower(-.1 + Math.abs(power) * power * .6);
//            myOpmode.telemetry.addData("power", -.1 + Math.abs(power) * power * .6);
//        }
//        if(buttonPressed && !lastButtonPressed){
//            if (open){
//                closeGrabber();
//                open = false;
//            }
//            else{
//                openGrabber();
//                open = true;
//            }
//        }
//        lastButtonPressed = buttonPressed;
//        myOpmode.telemetry.addData("position", arm.getCurrentPosition());
//        myOpmode.telemetry.update();
//    }
    //you actually only ever need the lift up method, not entirely sure why I had
    //a put down method

    public void liftUp(double position, double power){
        //double upPositon = startPos + 180;
        //may have to be negative, not entirely sure
        //probably want to either check if it is positive or negative.
        //also want to check if it is forward or reverse
        arm.setTargetPosition((int) -position);
        myOpmode.telemetry.addData("currentPos:", position);
        //don't give direct power, kinda troll
        //arm.setPower(.5);
        arm.setPower(power);
    }

    public void putDown(double position){
        //double downPosition = startPos;
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition((int) position);
        myOpmode.telemetry.addData("target:", position);
        //don't call power here
       // arm.setPower(.5);
    }
    public void stop(){
        arm.setPower(0);
    }
    public void setArmPower(double power){arm.setPower(power);}
    public double getStartPos(){
        return startPos;
    }
    public double getCurrentPos(){return arm.getCurrentPosition();}

    public void closeGrabber(){
        grabber.setPosition(.25);
    }

    public void openGrabber(){
        grabber.setPosition(.4);
    }
}
