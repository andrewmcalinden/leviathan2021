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

@Autonomous(name = "red auto", group = "18030")
public class RedAuto extends LinearOpMode {
    private Drivetrain dt;
    private Grabber grabber;

    public DcMotorEx mtrShooter;
    public DcMotor transfer;

    public Servo transferServo;

    public Vision ringCounter;

    public DcMotor intake;


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

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection((DcMotor.Direction.REVERSE));

        int numRings = 1;
        while(!isStarted()){
            //numRings = ringCounter.numRingsRightSide();
        }
        waitForStart();

        dt.movePIDFGyro(63, .8, 0, 0, .13, .25);
        dt.strafeGyro(.5, 20);
        dt.turnHeading(0, .000000000001, 0, 0, .11, .25); //sometimes innacurate
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
                dt.turnHeading(0, .000000000001, 0, 0, .11, .25); //sometimes innacurate
                dt.movePIDFGyro(17.5, .75, 0, 0, .14, .25);
                dt.turnHeading(90, .6, 0, 0, .14, .25);
                dt.movePIDFGyro(-13, .5, 0, .1, .14, .25);
                grabber.deployWobble();
                dt.movePIDFGyro(14.6, .5, 0, 0, .14, .25);
                //2nd wobble
                dt.turnHeading(90, .000000000001, 0, 0, .14, .25); //sometimes innacurate
                grabber.goToNeck();
                dt.strafePIDGyro(.65, 0.000001, .2, .14, -77, .5);
                dt.movePIDFGyro(-2, .3, 0, 0, .14, .25);
                dt.turnHeading(90, .000000000001, 0, 0, .11, .25); //sometimes innacurate
                sleep(500);
                grabber.closeGrabber();
                sleep(700);
                grabber.liftUp();
                grabber.holdUp();
                dt.turnHeading(90, .000000000001, 0, 0, .14, .25);
                dt.strafePIDGyro(.95, 0.000001, 0, .14, 69, .5);
                dt.movePIDFGyro(-12, .6, 0, 0, .14, .25);
                grabber.goToPos(675, .8);
                grabber.openGrabber();
                dt.movePIDFGyro(10, .5, 0, 0, .14, .25);
                dt.strafePIDGyro(.8, 0, 0, .14, 15, .5);
                break;
            case 1:
                dt.turnHeading(175, .8, 0, 0, .14, .25);
                dt.movePIDFGyro(-39, .9, 0, .8, .14, .25);
                grabber.deployWobble();
                sleep(200);
                grabber.goToStart();
                dt.turnHeading(180, .000000000001, 0, 0, .14, .25);
                dt.strafePIDGyro(.8, 0, 0, .14, -10.5, .5);
                dt.turnHeading(165, .2, 0, 0, .14, .25);
                intake.setPower(1);
                dt.movePIDFGyro(89, .8, 0, 0, .14, .25);
                intake.setPower(0);
                //2nd wobble
                dt.turnHeading(90, .8, 0, 0, .14, .25);
                grabber.goToNeck();
                dt.movePIDFGyro(-8.5, .35, 0, 0.3, .14, .25);
                sleep(500);
                grabber.closeGrabber();
                sleep(700);
                grabber.liftUp();
                grabber.holdUp();
                dt.turnHeading(174.9, .8, 0, 0, .14, .25);
                dt.movePIDFGyro(-75, .9, 0, .8, .14, .25);
                grabber.goToPos(675, .8);
                grabber.openGrabber();
                dt.movePIDFGyro(5, 1, 0, .8, .14, .25);
                break;
            default:
                dt.turnHeading(0, .3, 0, 0, .14, .25);
                dt.movePIDFGyro(67, .6, 0, 0, .14, .25);
                dt.turnHeading(90, .6, 0, 0, .14, .25);
                dt.movePIDFGyro(-10, .3, 0, 0, .14, .25);
                grabber.deployWobble();
                dt.movePIDFGyro(10, .4, 0, 0, .14, .25);
                dt.strafeGyro(.5, -50);
        }
        GlobalVars.finalHeading = dt.gyro.getAngle();
    }
}
