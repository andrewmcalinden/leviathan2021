package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.math.Vector;

public abstract class AdvancedLib extends OpMode {
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;

    public Sensors gyro;

    public DcMotor intake;

    public Grabber grabber;

    public DcMotorEx mtrShooter;
    public DcMotor transfer;

    public Servo transferServo;

    public boolean pressedALastTime;
    public boolean pressedBLastTime;
    public double mtrPower;

    public double servoPos;
    public boolean servoPressedLastTime;
    public double armStartPos;

    @Override
    public void init(){
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        //dont question reversals, they just work :)
        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);

        mtrShooter = hardwareMap.get(DcMotorEx.class, "shooter");
        mtrShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection((DcMotor.Direction.REVERSE));

        gyro = new Sensors(this);

        grabber = new Grabber(this);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        pressedALastTime = false;
        pressedBLastTime = false;
        mtrPower = 0;

        transferServo.setPosition(.45);

        servoPos = .45;
        servoPressedLastTime = false;
    }

    //angle must be measured counterclockwise from x axis
    public void fieldCentricMecanum(double x, double y, double turn, double robotHeadingRad){
        Vector movement = new Vector(x, y);
        movement = movement.rotated(-robotHeadingRad);
        double rightX = turn;

        double angle = movement.angle;
        double magnitude = movement.magnitude;

        double fl = magnitude * Math.sin(angle + Math.PI / 4) + rightX;
        double fr = magnitude * Math.sin(angle - Math.PI / 4) - rightX;
        double bl = magnitude * Math.sin(angle - Math.PI / 4) + rightX;
        double br = magnitude * Math.sin(angle + Math.PI / 4) - rightX;

        //only normalize if mag isnt 0 because if it is, we want to turn and will always be from 0-1
        if (magnitude != 0) {
            //Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(br));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(bl), max);

            //Divide everything by max (it's positive so we don't need to worry
            //about signs)
            //multiply by input magnitude as it represents true speed (from 0-1) that we want robot to move at
            fl = (fl / max) * magnitude;
            fr = (fr / max) * magnitude;
            bl = (bl / max) * magnitude;
            br = (br / max) * magnitude;
        }
//        telemetry.addData("fl: ", fl);
//        telemetry.addData("fr: ", fr);
//        telemetry.addData("bl: ", bl);
//        telemetry.addData("br ", br);
//        telemetry.addData("heading", robotHeadingRad);
//        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }

    public void updateTransfer(){
        if (gamepad2.x && !servoPressedLastTime){
            if (servoPos == 0){
                servoPos = .45;
                telemetry.clear();
                telemetry.addLine("safe to use servo :)");
                telemetry.update();
            }
            else{
                servoPos = 0;
                telemetry.addLine("DONT USE SERVO, YOU HAVE TO RESET IT");
                telemetry.update();
            }
            transferServo.setPosition(servoPos);
        }
        servoPressedLastTime = gamepad2.x;
    }

    public void robotCentricTrigMecanum(double x, double y, double turn){
        Vector movement = new Vector(x, y);
        double rightX = turn;

        double angle = movement.angle;
        double magnitude = movement.magnitude;

        double fl = magnitude * Math.sin(angle + Math.PI / 4) + rightX;
        double fr = magnitude * Math.sin(angle - Math.PI / 4) - rightX;
        double bl = magnitude * Math.sin(angle - Math.PI / 4) + rightX;
        double br = magnitude * Math.sin(angle + Math.PI / 4) - rightX;

        //only normalize if mag isnt 0 because if it is, we want to turn and will always be from 0-1
        if (magnitude != 0) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(br));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(bl), max);

            //Divide everything by max (it's positive so we don't need to worry
            //about signs)
            //multiply by input magnitude as it represents true speed (from 0-1) that we want robot to move at
            fl = (fl / max) * magnitude;
            fr = (fr / max) * magnitude;
            bl = (bl / max) * magnitude;
            br = (br / max) * magnitude;
        }
//        telemetry.addData("fl: ", fl);
//        telemetry.addData("fr: ", fr);
//        telemetry.addData("bl: ", bl);
//        telemetry.addData("br ", br);
//        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }

    public void robotCentricAdditiveMecanum(double x, double y, double turn){
        double magnitude = Math.hypot(x, y);

        double fl = y + turn + x;
        double fr = y - turn - x;
        double bl = y + turn - x;
        double br = y - turn + x;

        double max = 0;
        max = Math.max(Math.abs(fl), Math.abs(br));
        max = Math.max(Math.abs(fr), max);
        max = Math.max(Math.abs(bl), max);

        //only normalize if mag isnt 0 because if it is, we want to turn and will always be from 0-1
        if (magnitude != 0) {
            //Divide everything by max (it's positive so we don't need to worry
            //about signs)
            //multiply by input magnitude as it represents true speed (from 0-1) that we want robot to move at
            fl = (fl / max) * magnitude;
            fr = (fr / max) * magnitude;
            bl = (bl / max) * magnitude;
            br = (br / max) * magnitude;
        }
//        telemetry.addData("fl: ", fL.getCurrentPosition());
//        telemetry.addData("fr: ", fR.getCurrentPosition());
//        telemetry.addData("bl: ", bL.getCurrentPosition());
//        telemetry.addData("br ", bR.getCurrentPosition());
//        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }

    public void updateIntake(){
        if(gamepad1.left_bumper){
            intake.setPower(1);
            if (mtrPower == 0){
                transferServo.setPosition(.45);
            }
        }
        else if(gamepad1.dpad_down){
            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
        }
    }

    public void updateShooter(){
        if (gamepad2.a && !pressedALastTime){
            if (mtrPower == 0){
                mtrPower = 1;
            }
            else{
                mtrPower = 0;
            }
            mtrShooter.setPower(mtrPower);
            transfer.setPower(mtrPower == 0 ? 0 : 1);
        }
        else if (gamepad2.b && !pressedBLastTime){
            if (mtrPower == 0){
                mtrPower = .2;
            }
            else{
                mtrPower = 0;
            }
            mtrShooter.setPower(mtrPower);
            transfer.setPower(mtrPower == 0 ? 0 : 1);
        }
        pressedALastTime = gamepad2.a;
        pressedBLastTime = gamepad2.b;
    }

    public void updateShooterShaan(){
        if (gamepad1.right_bumper){
            mtrShooter.setPower(1);
            transfer.setPower(1);
        }
        else{
            mtrShooter.setPower(0);
            transfer.setPower(0);
        }
    }

    public void updateTransferShaan(){
        if (gamepad1.a && !servoPressedLastTime){
            if (servoPos == 0){
                servoPos = .45;
                telemetry.clear();
                telemetry.addLine("safe to use servo :)");
                telemetry.update();
            }
            else{
                servoPos = 0;
                telemetry.addLine("DONT USE SERVO, YOU HAVE TO RESET IT");
                telemetry.update();
            }
            transferServo.setPosition(servoPos);
        }
        servoPressedLastTime = gamepad1.a;
    }



    public void updateGrabber(){
        grabber.update(gamepad2.left_stick_y, gamepad2.y, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.right_bumper);
    }
}
