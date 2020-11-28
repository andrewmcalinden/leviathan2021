package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain  {

    public DcMotor fR;
    public DcMotor fL;
    public DcMotor bR;
    public DcMotor bL;

    LinearOpMode something;

    Sensors gyro;

    ElapsedTime clock;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void initv2(LinearOpMode opMode) {

        something = opMode;

        gyro.initv2(something);

        clock = new ElapsedTime();

        fR = something.hardwareMap.get(DcMotor.class, "fr");
        fL = something.hardwareMap.get(DcMotor.class, "fl");
        bR = something.hardwareMap.get(DcMotor.class, "br");
        bL = something.hardwareMap.get(DcMotor.class, "bl");

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
    }
        public void resetEncoder() {

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        something.telemetry.addLine("Encoders has been reset!");
        something.telemetry.update();
    }

    public void goStraight(double power, double inches) {
        resetEncoder();
        double startAngle = gyro.getAngle();
        while (Math.abs(getTic() / COUNTS_PER_INCH) < inches) {
            fR.setPower(power);
            fL.setPower(power);

        }
        stopMotors();
    }

    public void stopMotors() {
        fR.setPower(0);
        fL.setPower(0);

        something.telemetry.addLine("Motors has stopped");
        something.telemetry.update();
    }

    public double getTic() {
        double count = 2;
        if (fR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (fL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = fR.getCurrentPosition() + fL.getCurrentPosition();
        if (count == 0) {
            return 0;
        }

        something.telemetry.addData("Current tics",totaldis / count);
        something.telemetry.update();

        return totaldis / count;

    }

    public void startMotors(double leftPower, double rightPower) {
        fR.setPower(rightPower);
        fL.setPower(leftPower);

    }

    public void turnDeg(double deg, double power, boolean rigth) {
        double currentdeg = gyro.getAngle();
        double target = currentdeg + deg;
        if (rigth == true) {
            while (gyro.getAngle() < target) {
                fR.setPower(-power);
                fL.setPower(power);
            }
        }
        if (rigth == false) {
            while (gyro.getAngle() > target) {
                fR.setPower(power);
                fL.setPower(-power);
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





    public void trunPIDF(double angle, double p, double i, double d, double f){

        clock.reset();

        double Kp = p;
        double Ki = i;
        double Kd = d;

        double pastTime = 0;
        double currentTime = clock.milliseconds();
        double startAngle = gyro.getAngle();
        double angleDiff = angle - startAngle;
        double pastError = gyro.angleDiff(angle);
        double error = pastError;
        double power = 0;
        double integral = 0;

        while(Math.abs(error) > .6){
            error = gyro.angleDiff(angle);
            currentTime = clock.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / angleDiff;
            integral += dt * ((error + pastError) / 2);
            double derivative = (error - pastError) / dt;

            power = Kp * proportional + Ki * integral + Kd * derivative;

            if(power < 0){
                startMotors(power - f,-power + f);
            }
            else{
                startMotors(power + f,-power - f);
            }

            pastError = error;
            pastTime = currentTime;

        }

        stopMotors();
    }

    public void movePIDFGyro(double inches, double p, double i, double d, double f){
        resetEncoder();
        clock.reset();

        double Kp = p;
        double Ki = i;
        double Kd = d;

        double pastTime = 0;
        double currentTime = clock.milliseconds();
        double startPosition = getTic();
        double diffInches = inches - startPosition;
        double pastError = inches;
        double error = pastError;
        double power = 0;
        double integral = 0;
        double initialAngle = gyro.getAngle();

        while(Math.abs(error) > .6){
            error = inches - Math.abs(getTic() / COUNTS_PER_INCH);
            currentTime = clock.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / diffInches;
            integral += dt * ((error + pastError) / 2);
            double derivative = (error - pastError) / dt;

            power = Kp * proportional + Ki * integral + Kd * derivative;
            double difference = Math.abs(gyro.angleDiff(initialAngle));
            if(difference > 2) {
                if(gyro.angleDiff(initialAngle) < 0){
                    startMotors(power + f, (power + f) * .8);
                }
                else{
                    startMotors((power + f) * .8, power + f);
                }
            }
            else{
                startMotors(power + f, power + f);
            }
            pastError = error;
            pastTime = currentTime;

        }
        resetEncoder();
        stopMotors();
    }

}