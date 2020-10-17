package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//controller wire
//AA batteries
//gobilda servo programmer
//plastic tubs

@TeleOp(name = "Teleop", group = "18030")
public class Teleop extends LinearOpMode{

    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;

    public Sensors gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while(!isStopRequested()){
            double robotHeadingRad = gyro.getAngle() * (180 / Math.PI);
            //might need to negate left stick y because apparently that's a thing
            fieldCentricMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeadingRad);

        }
    }

    public void initHardware(){
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

        fR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);

        gyro = new Sensors(this);
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
        telemetry.addData("fl: ", fl);
        telemetry.addData("fr: ", fr);
        telemetry.addData("bl: ", bl);
        telemetry.addData("br ", br);
        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
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
        telemetry.addData("fl: ", fl);
        telemetry.addData("fr: ", fr);
        telemetry.addData("bl: ", bl);
        telemetry.addData("br ", br);
        telemetry.update();

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
        telemetry.addData("fl: ", fl);
        telemetry.addData("fr: ", fr);
        telemetry.addData("bl: ", bl);
        telemetry.addData("br ", br);
        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(bl);
        bR.setPower(br);
    }
}