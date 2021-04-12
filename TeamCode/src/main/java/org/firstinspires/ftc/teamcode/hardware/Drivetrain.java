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

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
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
    }

    public void strafePIDGyro(double kp, double ki, double kd, double f, double inches, double threshold, double time){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (timeAtSetPoint < time && !opMode.isStopRequested()) {
            if (inches < 0){
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            double difference = gyro.angleDiff(initialHeading);

            if (difference > .5){
                if (power > 0) {
                    startMotors(.8 * (power + f), .8 * (-power - f), 1.2 * (-power - f), 1.2 * (power + f));

                }
                else {
                    startMotors(1.2 * (power - f), 1.2 * (-power + f), .8 * (-power + f), .8 * (power - f));
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    startMotors(1.2 * (power + f), 1.2 * (-power - f), .8 * (-power - f), .8 * (power + f));

                }
                else {
                    startMotors(.8 * (power - f), .8 * (-power + f), 1.2 * (-power + f), 1.2 * (power - f));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f, -power - f, -power - f, power + f);

                }
                else {
                    startMotors(power - f, -power + f, -power + f, power - f);
                }
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }

    public void stopMotors() {
        fR.setPower(0);
        fL.setPower(0);
        bR.setPower(0);
        bL.setPower(0);
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
        return totaldis / count;
    }

    public void startMotors(double fl, double fr, double bl, double br) {
        fR.setPower(fr);
        fL.setPower(fl);
        bL.setPower(bl);
        bR.setPower(br);

        opMode.telemetry.addData("fl", fl);
        opMode.telemetry.addData("fr", fr);
        opMode.telemetry.addData("bl", bl);
        opMode.telemetry.addData("br", br);
        opMode.telemetry.update();
    }

    double angleWrapDeg(double angle) {
        double correctAngle = angle;
        while (correctAngle > 180)
        {
            correctAngle -= 360;
        }
        while (correctAngle < -180)
        {
            correctAngle += 360;
        }
        return correctAngle;
    }

    public void turnHeading(double finalAngle, double kp, double ki, double kd, double f, double threshold, double time) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleWrapDeg(initialHeading - finalAngle);
        double error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (!opMode.isStopRequested() && timeAtSetPoint < time) {
            error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / Math.abs(initialAngleDiff);
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            if (power > 0) {
                startMotors(-power - f, power + f, -power - f, power + f);
            }
            else{
                startMotors(-power + f, power - f, -power + f, power - f);
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }

    public void movePIDFGyro(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;


        while (timeAtSetPoint < time && !opMode.isStopRequested()) {
            if (inches < 0){
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }
            opMode.telemetry.addData("error", error);

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();
            double difference = gyro.angleDiff(initialHeading);

            if (difference > .4){
                if (power > 0) {
                    startMotors((power + f) * .8, 1.2 * (power + f), (power + f) * .8, 1.2 * (power + f));
                }
                else {
                    startMotors((1.2 * (power - f)), (power - f) * .8, 1.2 * ((power - f)), (power - f) * .8);
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    startMotors(1.2 * (power + f), (power + f) * .8, 1.2 * (power + f), (power + f) * .8);
                }
                else {
                    startMotors((power - f) * .8, (1.2 * (power - f)), (power - f) * .8, (1.2 * (power - f)));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f, power + f, power + f, power + f);
                }
                else {
                    startMotors(power - f, power - f, power - f, power - f);
                }
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }
}