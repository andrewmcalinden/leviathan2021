package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.FinalHeading;

@Autonomous(name = "blue wobble mid", group = "18030")
public class BlueMidGoalWobble extends LinearOpMode {
    private Drivetrain dt;
    private Grabber grabber;

    public DcMotorEx mtrShooter;
    public DcMotor transfer;

    public Servo transferServo;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);
        grabber = new Grabber(this);

        mtrShooter = hardwareMap.get(DcMotorEx.class, "shooter");
        mtrShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrShooter.setVelocityPIDFCoefficients(2.87442, 0.287442, 0, 28.7442);

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        grabber.closeGrabber();
        transferServo.setPosition(.35);
        waitForStart();

        if (!isStopRequested()){
            dt.movePIDFGyro(63, .8, 0, 0, .1);
            dt.strafeInches(.5, 30);
            for (int i = 0; i < 10; i++){
                mtrShooter.setVelocity(1380);
                transfer.setPower(1);
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                while (timer.seconds()  < 2){
                    //do nothing
                    telemetry.addData("velocity", mtrShooter.getVelocity());
                    telemetry.update();
                }
                transferServo.setPosition(0);
                sleep(300);
                transferServo.setPosition(.35);
                telemetry.addData("velocity", mtrShooter.getVelocity());
                telemetry.update();
            }
            mtrShooter.setVelocity(0);
            transfer.setPower(0);
            dt.movePIDFGyro(8, .4, 0, 0, .1);
            FinalHeading.finalHeading = dt.gyro.getAngle();
            stop();
        }
    }
}
