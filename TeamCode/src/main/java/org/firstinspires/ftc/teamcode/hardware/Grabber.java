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

    public double startPos;
    private boolean lastButtonPressed;
    private boolean open;

    public Grabber(LinearOpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection((DcMotor.Direction.REVERSE));
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        grabber = opMode.hardwareMap.servo.get("grabber");
        closeGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        myOpmode = opMode;
    }

    public Grabber(OpMode opMode){
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection((DcMotor.Direction.REVERSE));
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = opMode.hardwareMap.servo.get("grabber");
        openGrabber();
        startPos = arm.getCurrentPosition();
        lastButtonPressed = false;
        open = false;

        myOpmode = opMode;
    }

    public void setStartPos(double position){
        startPos = position;
    }

    //this is some cheese, should probably comment out
    public void update(double power, boolean buttonPressed, boolean liftUp, boolean goToNeck, boolean hold){
        if(hold){
            arm.setPower(-.2);
        }
        else if (power == 0){
            arm.setPower(0);
            if (liftUp){
                liftUp();
            }
            if (goToNeck){
                goToNeck();
            }
        }
        else{
            arm.setPower(Math.abs(power) * Math.abs(power) * power * .6);
        }
        if(buttonPressed && !lastButtonPressed){
            if (open){
                closeGrabber();
                open = false;
                myOpmode.telemetry.addLine("closing");
            }
            else{
                openGrabber();
                open = true;
                myOpmode.telemetry.addLine("opening");
            }
            myOpmode.telemetry.addLine("button pressed");
            myOpmode.telemetry.update();
        }
//        myOpmode.telemetry.addData("position", arm.getCurrentPosition());
////        myOpmode.telemetry.addData("power",-.1 + Math.abs(power) * Math.abs(power) * power );
//        myOpmode.telemetry.update();
        lastButtonPressed = buttonPressed;
    }

    public void goToPos(double target, double power){
        //myOpmode.telemetry.addData("initialPos", startPos);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        target += startPos;
//        myOpmode.telemetry.addData("target", target);
//        myOpmode.telemetry.update();
        double error = target - arm.getCurrentPosition(); //-500
        double initialError = Math.abs(error); //500
        while(Math.abs(error) > 15 && timer.seconds() < 2){
            error = target - arm.getCurrentPosition();
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * power + f);
        }
        arm.setPower(0);
    }

    public void liftUp(){
        goToPos(350, .75);
    }

    public void goToStart(){
        goToPos(startPos, .6);
        goToPos(startPos, .6);
    }

    public void goToNeck(){
        goToPos(675, .6);
    }

    public void deployWobble(){
        goToPos(750, .6);
        openGrabber();
    }

    public void closeGrabber(){
        grabber.setPosition(.5);
    }

    public void openGrabber(){
        grabber.setPosition(.33);
    }
}
