package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Teleop extends LinearOpMode{
    Drivetrain drive;
    DcMotor motor;
    Arm outTake;
    Sensors gyro;
    boolean a;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.initv2(this);
        outTake.initv2(this);
        motor = hardwareMap.get(DcMotor.class, "motor");
        gyro.initv2(this);
        a = false;

        while(!isStopRequested()){
            fieldCentricMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gyro.getAngle() * (180 / Math.PI));

            if(gamepad2.a == true){
                a = true;
            }

            if(gamepad2.x == true){
                a = false;
            }

            if(a == true){
                motor.setPower(1);
            }
            else{
                motor.setPower(0);
            }

            if(gamepad2.b == true){
                outTake.openGate();
            }

            if(gamepad2.y == true){
                outTake.closeGate();
            }


        }
    }

    public void arcadeDrive(){
        double power = gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
        double turningpower = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        double leftpower = power + turningpower;
        double rightpower = power - turningpower;
        boolean a = false;

        if(Math.abs(leftpower) > 1) {
            double oldleftpower = leftpower;
            leftpower/=leftpower;
            rightpower/=oldleftpower;
        }
        else if(Math.abs(rightpower) > 1){
            double oldrightpower = rightpower;
            rightpower/=rightpower;
            leftpower/=oldrightpower;
        }

        drive.startMotors(leftpower,rightpower);
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
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl), Math.abs(br));
            max = Math.max(Math.abs(fr), max);
            max = Math.max(Math.abs(bl), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            //multiply by input magnitude as it represents true speed (from 0-1) that we want robot to move at
            fl = (fl / max) * magnitude;
            fr = (fr / max) * magnitude;
            bl = (bl / max) * magnitude;
            br = (br / max) * magnitude;
        }
        System.out.println("fl: " + fl);
        System.out.println("fr: " + fr);
        System.out.println("bl: " + bl);
        System.out.println("br " + br);

        drive.fL.setPower(fl);
        drive.fR.setPower(fr);
        drive.bL.setPower(bl);
        drive.bR.setPower(br);
    }
}
