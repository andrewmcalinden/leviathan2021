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
            arm.setPower(-.1 + Math.abs(power) * Math.abs(power) * power);
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
        myOpmode.telemetry.addData("position", arm.getCurrentPosition());
        myOpmode.telemetry.addData("power",-.1 + Math.abs(power) * Math.abs(power) * power );
        myOpmode.telemetry.update();
        lastButtonPressed = buttonPressed;
    }

    public void liftUp(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos + 300; //500
        double error = target - arm.getCurrentPosition(); //-500
        double initialError = Math.abs(error); //500
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = target - arm.getCurrentPosition();
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .25 + f);
            myOpmode.telemetry.addData("power", p * .25 + f);
            myOpmode.telemetry.addData("error", error);
            myOpmode.telemetry.update();
        }
        myOpmode.telemetry.clear();
        myOpmode.telemetry.update();
        arm.setPower(0);
    }

    public void goToNeck(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos + 600;
        double error = target - arm.getCurrentPosition(); //-500
        double initialError = Math.abs(error);
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = target - arm.getCurrentPosition(); //-500
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .25 + f); //was -.6
        }
        arm.setPower(0);
    }

    public void deployWobble(){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double target = startPos + 700; //-300
        double error = target - arm.getCurrentPosition(); //-500
        double initialError = Math.abs(error); //130
        while(Math.abs(error) > 5 && timer.seconds() < 2){
            error = target - arm.getCurrentPosition(); //-50
            double p = error / initialError; //will be positive if starting from initialization
            double f = p > 0 ? .07 : -.07;
            arm.setPower(p * .45  + f); //was -.6
        }
        openGrabber();
        arm.setPower(0);
    }

    public void closeGrabber(){
        grabber.setPosition(.4);
    }

    public void openGrabber(){
        grabber.setPosition(.25);
    }
}
