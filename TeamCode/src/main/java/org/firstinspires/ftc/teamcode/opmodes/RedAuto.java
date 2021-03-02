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

@Autonomous(name = "red auto", group = "18030")
public class RedAuto extends LinearOpMode {
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

        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        grabber.closeGrabber();
        transferServo.setPosition(.45);
        waitForStart();

        while (!isStopRequested()){
            dt.movePIDFGyro(63, .8, 0, 0, .14);
            dt.strafeGyro(.5, 20);
            dt.turnHeading(20, .3, 0, 0, .14);
            for (int i = 0; i < 4; i++){
                mtrShooter.setPower(1);
                transfer.setPower(1);
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                while (timer.seconds() < 2 && !isStopRequested()){
                    //do nothing
                    telemetry.addLine("we chilling");
                    telemetry.update();
                }
                transferServo.setPosition(0);
                sleep(300);
                transferServo.setPosition(.45);
            }
            mtrShooter.setPower(0);
            transfer.setPower(0);

            dt.turnHeading(0, .3, 0, 0, .14); //turn to 0 degrees

            //hopefully we land in zone b
            dt.movePIDFGyro(43, .6, 0, 0, .14);

            //case A
            dt.strafeGyro(.5, 30);
            dt.turnHeading(0, .3, 0, 0, .14); //turn to 0 degrees
            grabber.deployWobble();
            sleep(500);
            dt.strafeGyro(.5, -30);
            dt.movePIDFGyro(-30, .5, 0, 0, .14);

            /*
            Cheese way to do it without turns or backing up(try this out if you want)
            Case 0 (A):
                dt.strafeInches(.5, 30)
                grabber.deployWobble();
                dt.strafeInches(-.5, 30)
                dt.goStraight(-.5, 30)
            Case 1(B):
                dt.turnPIDF(-180, .3, 0, 0, .1);
                grabber.deployWobble();
                dt.movePIDFGyro(30, .4, 0, 0, .1);
            Case 4(C):
                dt.movePIDFGyro(6, .4, 0, 0, .1);
                dt.turnPIDF(-180, .3, 0, 0, .1);
                dt.strafeInches(-.5, 30)
                grabber.deployWobble();
                dt.movePIDFGyro(36, .4, 0, 0, .1);
            */

            FinalHeading.finalHeading = dt.gyro.getAngle();
            break;
        }
        stop();
    }
}
