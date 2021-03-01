package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.testing.TestAnything;

public class Drivetrain  {

    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bR;
    public DcMotor bL;

    LinearOpMode opMode;

    public Sensors gyro;

    ElapsedTime timer;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Drivetrain(LinearOpMode opMode) throws InterruptedException{

        this.opMode = opMode;

        gyro = new Sensors(opMode);

        timer = new ElapsedTime();

        fR = this.opMode.hardwareMap.get(DcMotor.class, "fR");
        fL = this.opMode.hardwareMap.get(DcMotor.class, "fL");
        bR = this.opMode.hardwareMap.get(DcMotor.class, "bR");
        bL = this.opMode.hardwareMap.get(DcMotor.class, "bL");

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //dont question reversals, they just work :)
        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void resetEncoder() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

        opMode.telemetry.addLine("Encoders has been reset!");
        opMode.telemetry.update();
    }

    public void goStraight(double power, double inches){
        resetEncoder();
        while (Math.abs(getTic() / COUNTS_PER_INCH) < inches && !opMode.isStopRequested()) {
            startMotors(power, power, power, power);
            opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
            opMode.telemetry.update();
        }
        stopMotors();
    }

    public void strafeInches(double power, double inches){
        resetEncoder();
        while (Math.abs(getTicStrafeRight() / COUNTS_PER_INCH) < inches && !opMode.isStopRequested()) {
            startMotors(power, -power, -power, power);
            opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
            opMode.telemetry.update();
        }
        stopMotors();
    }


    public void stopMotors() {
        fR.setPower(0);
        fL.setPower(0);
        bR.setPower(0);
        bL.setPower(0);

        opMode.telemetry.addLine("Motors has stopped");
        opMode.telemetry.update();
    }

    public double getTic() {
        double count = 4;
        if (fR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (fL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = Math.abs(fR.getCurrentPosition()) + Math.abs(fL.getCurrentPosition()) + Math.abs(bL.getCurrentPosition()) + Math.abs(bR.getCurrentPosition());
        if (count == 0) {
            return 1;
        }

        opMode.telemetry.addData("Current tics",totaldis / count);
        opMode.telemetry.update();

        return totaldis / count;
    }

    public double getTicStrafeRight(){
        double count = 4;
        if (fR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (fL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (bL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = -fR.getCurrentPosition() + fL.getCurrentPosition() - bL.getCurrentPosition() + bR.getCurrentPosition();
        if (count == 0) {
            return 1;
        }

        opMode.telemetry.addData("Current tics",totaldis / count);
        opMode.telemetry.update();

        return totaldis / count;
    }

    public void startMotors(double fl, double fr, double bl, double br) {
        fR.setPower(fr);
        fL.setPower(fl);
        bL.setPower(-bl);
        bR.setPower(br);

    }

    public void turnDeg(double deg, double power, boolean right) {
        double currentdeg = gyro.getAngle();
        double target = currentdeg + deg;
        if (right == true) {
            while (gyro.getAngle() < target) {
                fR.setPower(-power);
                bR.setPower(-power);
                fL.setPower(power);
                bL.setPower(power);
            }
        }
        if (right == false) {
            while (gyro.getAngle() > target) {
                fR.setPower(power);
                bR.setPower(power);
                fL.setPower(-power);
                bL.setPower(-power);
            }
        }
        stopMotors();
    }

    public void turnHeading(double heading, double power) {
        double distance = gyro.angleDiff(heading);
        if (distance > 0) {
            turnDeg(distance, power, true);
        } else {
            turnDeg(distance, power, false);
        }
    }

    public void turnPIDF(double angle, double p, double i, double d, double f){
        timer.reset();

        double Kp = p;
        double Ki = i;
        double Kd = d;

        double pastTime = 0;
        double currentTime = timer.milliseconds();
        double startAngle = gyro.getAngle();
        double angleDiff = angle - startAngle;
        double pastError = gyro.angleDiff(angle);
        double error = pastError;
        double power = 0;
        double integral = 0;

        while(Math.abs(error) > .6){
            error = gyro.angleDiff(angle);
            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / angleDiff;
            integral += dt * ((error + pastError) / 2);
            double derivative = (error - pastError) / dt;

            power = Kp * proportional + Ki * integral + Kd * derivative;

            if(power < 0){
                startMotors(power - f,-power + f, power - f,-power + f);
            }
            else{
                startMotors(power + f,-power - f, power + f,-power - f);
            }
            pastError = error;
            pastTime = currentTime;
        }
        stopMotors();
    }

    //doesnt work, need to take into account going backwards
    public void movePIDFGyro(double inches, double p, double i, double d, double f){
        resetEncoder();
        timer.reset();

        double Kp = p;
        double Ki = i;
        double Kd = d;

        double pastTime = 0;
        double currentTime = timer.milliseconds();
        double startPosition = getTic() / COUNTS_PER_INCH;
        double diffInches = inches - startPosition;
        double pastError = inches;
        double error = pastError;
        double power = 0;
        double integral = 0;
        double initialAngle = gyro.getAngle();

        while(Math.abs(error) > .6 && !opMode.isStopRequested()){
            error = inches - getTic() / COUNTS_PER_INCH;
            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / diffInches;
            integral += dt * ((error + pastError) / 2);
            double derivative = (error - pastError) / dt;

            power = Kp * proportional + Ki * integral + Kd * derivative;
            double difference = Math.abs(gyro.angleDiff(initialAngle));
            if(difference > 2) {
                if(gyro.angleDiff(initialAngle) < 0){
                    startMotors(power + f, (power + f) * .8, power + f, (power + f) * .8);
                }
                else{
                    startMotors((power + f) * .8, power + f, (power + f) * .8, power + f);
                }
            }
            else{
                startMotors(power + f, power + f, power + f, power + f);
            }
            pastError = error;
            pastTime = currentTime;
        }
        resetEncoder();
        stopMotors();
    }
}