package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection((DcMotor.Direction.FORWARD));
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        myOpmode = opMode;
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

        myOpmode = opMode;
    }
    //this is some cheese, should probably comment out
    public void update(double power, boolean buttonPressed, boolean liftUp, boolean goToNeck){
        if (power == 0){
            arm.setPower(0);
            if (liftUp){
                liftUp();
                myOpmode.telemetry.addLine("going up");
                myOpmode.telemetry.update();
            }
            if (goToNeck){
                goToNeck();
                myOpmode.telemetry.addLine("going to neck");
                myOpmode.telemetry.update();
            }
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

    public void liftUp(){ //-16 = start, -144 = up, -270 = neck
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos - 130; //-130
        double error = arm.getCurrentPosition() - target;
        double initialError = Math.abs(error); //130
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = arm.getCurrentPosition() - target; //positive, starts at 130, goes closer to 0 over time
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .25 + f); //was -.6
        }
        arm.setPower(0);
    }

    public void goToNeck(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos - 250; //-250
        double error = arm.getCurrentPosition() - target;
        double initialError = Math.abs(error); //130
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = arm.getCurrentPosition() - target; //positive, starts at 130, goes closer to 0 over time
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .25 + f); //was -.6
        }
        arm.setPower(0);
    }

    public void deployWobble(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos - 300; //-300
        double error = arm.getCurrentPosition() - target;
        double initialError = Math.abs(error); //130
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = arm.getCurrentPosition() - target; //positive, starts at 130, goes closer to 0 over time
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .45  + f); //was -.6
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
