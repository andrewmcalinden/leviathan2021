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

    public void goStraight(double power, double inches){
        resetEncoder();
        while (Math.abs(getTic() / COUNTS_PER_INCH) < inches && !opMode.isStopRequested()) {
            startMotors(power, power, power, power);
            opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
            opMode.telemetry.update();
        }
        stopMotors();
    }

    public void strafeInches(double power, double inches){ //goes right
        resetEncoder();
        while (Math.abs(getTic() / COUNTS_PER_INCH) < inches && !opMode.isStopRequested()) {
            startMotors(power, -power, -power, power);
            opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
            opMode.telemetry.update();
        }
        stopMotors();
    }

    //maybe add pid later
    public void strafeGyro(double power, double inches){
        double initialAngle = gyro.getAngle();
        resetEncoder();
        if (inches > 0){ //strafe right
            while (Math.abs(getTic() / COUNTS_PER_INCH) < inches && !opMode.isStopRequested()) {
                double angleDiff = gyro.getAngle() - initialAngle;
                if (angleDiff > 1){
                    startMotors(power * 1.2, -power * 1.2, -power * .8, power * .8);
                    opMode.telemetry.addData("turning left", gyro.getAngle());
                }
                else if (angleDiff < -1){
                    startMotors(power * .8, -power * .8, -power * 1.2, power * 1.2);
                    opMode.telemetry.addData("turning right", gyro.getAngle());
                }
                else{
                    startMotors(power, -power, -power, power);
                }

                opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
                opMode.telemetry.update();
            }
        }
        else{ //strafe left
            while (Math.abs(getTic() / COUNTS_PER_INCH) < Math.abs(inches) && !opMode.isStopRequested()) {
                double angleDiff = gyro.getAngle() - initialAngle;
                if (angleDiff < -1){
                    startMotors(-power * 1.2, power * 1.2, power * .8, -power * .8);
                    opMode.telemetry.addData("turning left", gyro.getAngle());
                }
                else if (angleDiff > 1){
                    startMotors(-power * .8, power * .8, power * 1.2, -power * 1.2);
                    opMode.telemetry.addData("turning right", gyro.getAngle());
                }
                else{
                    startMotors(-power, power, power, -power);
                }
                opMode.telemetry.addData("position", getTic() / COUNTS_PER_INCH);
                opMode.telemetry.update();
            }
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

    public void turnHeading(double finalAngle, double kp, double ki, double kd, double f) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleWrapDeg(initialHeading - finalAngle);
        double error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);
        double pastError = error;

        double integral = 0;

        while (Math.abs(error) > 1 && !opMode.isStopRequested()) {
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
            else {
                startMotors(-power + f, power - f, -power + f, power - f);
            }
            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }

    public void movePIDFGyro(double inches, double kp, double ki, double kd, double f){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        while (Math.abs(error) > 1 && !opMode.isStopRequested()) {
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
            double difference = Math.abs(gyro.angleDiff(initialHeading));

            if (difference > 2){
                if (power > 0) {
                    startMotors((power + f) * .8, power + f, (power + f) * .8, power + f);
                    //opMode.telemetry.addLine("setting positive powers");

                }
                else {
                    startMotors((power - f), (power - f) * .8, (power - f), (power - f) * .8);
                    //opMode.telemetry.addLine("setting negative powers 1");
                }
            }
            else if(difference < -2){
                if (power > 0) {
                    startMotors(power + f, (power + f) * .8, power + f, (power + f) * .8);
                    //opMode.telemetry.addLine("setting positive powers");

                }
                else {
                    startMotors((power - f) * .8, (power - f), (power - f) * .8, (power - f));
                    //opMode.telemetry.addLine("setting negative powers 2");

                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f, power + f, power + f, power + f);
                    //opMode.telemetry.addLine("setting positive powers");

                }
                else {
                    startMotors(power - f, power - f, power - f, power - f);
                    //opMode.telemetry.addLine("setting negative powers 3");
                }
            }
            pastTime = currentTime;
            pastError = error;
            //opMode.telemetry.update();
        }
        stopMotors();
    }

    //only works for going forwards
    public void oldMovePIDFGyro(double inches, double p, double i, double d, double f){
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
            double tic = inches > 0 ? getTic() : -getTic();
            error = inches - tic / COUNTS_PER_INCH; //if math.abs, probably would fix
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