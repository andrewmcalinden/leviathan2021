package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.GlobalVars;

@Autonomous(name = "red single wobble", group = "18030")
public class RedSingleWobble extends LinearOpMode {
    private Drivetrain dt;
    private Grabber grabber;

    public DcMotorEx mtrShooter;
    public DcMotor transfer;

    public Servo transferServo;

    public Vision ringCounter;

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

        telemetry.setAutoClear(false);
        ringCounter = new Vision(this);

        int numRings = 0;
        while(!isStarted()){
            numRings = ringCounter.numRingsRightSide();
        }

        waitForStart();

        while (!isStopRequested()){
            dt.movePIDFGyro(63, .8, 0, 0, .13);
            dt.strafeGyro(.5, 20);
            dt.turnHeading(0, .000000000001, 0, 0, .11); //sometimes innacurate
            for (int i = 0; i < 3; i++){
                mtrShooter.setPower(1);
                transfer.setPower(1);
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                while (timer.milliseconds() < 1400 && !isStopRequested()){
                    //do nothing
                    telemetry.addLine("we chilling");
                    telemetry.update();
                }
                transferServo.setPosition(0);
                sleep(300);
                transferServo.setPosition(.45);
            }
            sleep(600);
            mtrShooter.setPower(0);
            transfer.setPower(0);
            switch (numRings){
                case 0:
                    dt.turnHeading(0, .000000000001, 0, 0, .11); //sometimes innacurate
                    dt.movePIDFGyro(17.5, .75, 0, 0, .14);
                    dt.turnHeading(90, .6, 0, 0, .14);
                    dt.movePIDFGyro(-13, .5, 0, .1, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(14.6, .5, 0, 0, .14);
                    //2nd wobble
                    dt.turnHeading(90, .000000000001, 0, 0, .14); //sometimes innacurate
                    grabber.goToNeck();
                    dt.strafePIDGyro(.65, 0.000001, .2, .14, -77);
                    dt.turnHeading(90, .000000000001, 0, 0, .11); //sometimes innacurate
                    sleep(500);
                    grabber.closeGrabber();
                    sleep(700);
                    grabber.liftUp();
                    grabber.holdUp();
                    dt.turnHeading(90, .000000000001, 0, 0, .14);
                    dt.strafePIDGyro(.95, 0.000001, 0, .14, 69);
                    dt.movePIDFGyro(-12, .6, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(10, .5, 0, 0, .14);
                    dt.strafePIDGyro(.8, 0, 0, .14, 15);
                    break;
                case 1:
                    dt.turnHeading(180, .8, 0, 0, .14);
                    dt.movePIDFGyro(-43, .9, 0, 0, .14);
                    grabber.deployWobble();
                    sleep(200);
                    grabber.goToStart();
                    dt.turnHeading(180, .000000000001, 0, 0, .14);
                    dt.strafePIDGyro(.3, 0, 0, .14, 11.5);
                    dt.movePIDFGyro(85.5, .8, 0, 0, .14);
                    //2nd wobble
                    dt.turnHeading(90, .8, 0, 0, .14);
                    grabber.goToNeck();
                    dt.movePIDFGyro(-9.3, .2, 0, 0, .14);
                    sleep(200);
                    grabber.closeGrabber();
                    sleep(500);
                    grabber.liftUp();
                    grabber.holdUp();
                    dt.movePIDFGyro(2, .8, 0, 0, .14);
                    dt.turnHeading(180, .8, 0, 0, .14);
                    dt.movePIDFGyro(-75, .8, 0, 0, .14);
                    dt.strafePIDGyro(.3, 0, 0, .14, -8);
                    grabber.deployWobble();
                    dt.movePIDFGyro(5, .7, 0, 0, .14);
                    break;
                default:
                    dt.turnHeading(0, .3, 0, 0, .14);
                    dt.movePIDFGyro(67, .6, 0, 0, .14);
                    dt.turnHeading(90, .6, 0, 0, .14);
                    dt.movePIDFGyro(-10, .3, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(10, .4, 0, 0, .14);
                    dt.strafeGyro(.5, -50);
            }
            GlobalVars.finalHeading = dt.gyro.getAngle();
            break;
        }
        stop();
    }
}
