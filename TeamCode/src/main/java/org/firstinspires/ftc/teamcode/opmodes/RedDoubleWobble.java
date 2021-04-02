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

@Autonomous(name = "red double wobble", group = "18030")
public class RedDoubleWobble extends LinearOpMode {
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

        numRings = 0;
        waitForStart();

        while (!isStopRequested()){
            dt.movePIDFGyro(63, .8, 0, 0, .14);
            dt.strafePIDGyro(.8, 0, 0, .14, 20);
            dt.turnHeading(7, .3, 0, 0, .14);
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
                sleep(400);
                transferServo.setPosition(.45);
            }
            sleep(300);
            mtrShooter.setPower(0);
            transfer.setPower(0);

            switch (numRings){
                case 0:
                    dt.movePIDFGyro(17, .75, 0, 0, .14);
                    dt.turnHeading(90, .6, 0, 0, .14);
                    dt.movePIDFGyro(-17, .5, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(18, .5, 0, 0, .14);
                    //2nd wobble
                    dt.turnHeading(90, .1, 0, 0, .14);
                    dt.strafePIDGyro(.8, 0.000001, 0, .14, -77.7);
                    //dt.turnHeading(90, .2, 0, 0, .14);
                    grabber.goToNeck();
                    sleep(500);
                    dt.movePIDFGyro(-9.7, .6, 0, 0, .14);
                    sleep(500);
                    grabber.closeGrabber();
                    sleep(500);
                    grabber.liftUp();
                    grabber.holdUp();
                    dt.turnHeading(90, .1, 0, 0, .14);
                    dt.movePIDFGyro(5, .5, 0, 0, .14);
                    dt.strafePIDGyro(.95, 0.000001, 0, .14, 67);
                    dt.movePIDFGyro(-19.5, .6, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(10, .5, 0, 0, .14);
                    dt.strafePIDGyro(.8, 0, 0, .14, 15);
                    break;
                case 1:
                    /* old single wobble
                    dt.turnHeading(180, .8, 0, 0, .14);
                    dt.movePIDFGyro(-37, .6, 0, 0, .14);
                    dt.strafeGyro(.3, -5);
                    grabber.deployWobble();
                    dt.movePIDFGyro(20, .6, 0, 0, .14);
                    */
                    //new stuff
                    //depositing the first wobble
                    dt.movePIDFGyro(40, .6,0,0,.14);
                    dt.turnHeading(-90, .1, 0, 0, .14);
                    //going to want to tune this distance
                    dt.movePIDFGyro(16, .2, 0, 0, .14);
                    grabber.deployWobble();

                    //turning so that it's zero here aligns the back of the bot(arm) with the goal
                    dt.turnHeading(0, .1, 0, 0, .14);
                    //this distance too
                    dt.movePIDFGyro(-80, .65, 0, 0, .14);
                    //when grabbing, I honestly think you only need a sleep of like 250 milliseconds, right now you sleep for basically 1.5 seconds
                    grabber.goToNeck();
                    sleep(250);
                    dt.movePIDFGyro(-9.7, .6, 0, 0, .14);
                    sleep(250);
                    grabber.closeGrabber();
                    sleep(250);
                    grabber.liftUp();
                    grabber.holdUp();
                    //tune this distance
                    dt.movePIDFGyro(76, .65, 0, 0, .14);
                    dt.turnHeading(-90, .1, 0, 0, .14);
                    grabber.deployWobble();
                    //tune this as well
                    dt.strafePIDGyro(.8, 0, 0, .14, 15);
                    break;
                default:
                    dt.turnHeading(0, .3, 0, 0, .14);
                    dt.movePIDFGyro(67, .6, 0, 0, .14);
                    dt.turnHeading(90, .6, 0, 0, .14);
                    dt.movePIDFGyro(-10, .3, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(10, .4, 0, 0, .14);
                    //dt.strafeGyro(.5, -50);
                    //2nd wobble
                    dt.strafeGyro(.5, -110);
                    dt.movePIDFGyro(-5, .3, 0, 0, .14);
                    grabber.goToNeck();
                    sleep(200);
                    grabber.closeGrabber();
                    sleep(200);
                    grabber.liftUp();
                    dt.movePIDFGyro(5, .3, 0, 0, .14);
                    dt.strafeGyro(.5, 60);
            }
            GlobalVars.finalHeading = dt.gyro.getAngle();
            //grabber.goToStart();
            break;
        }
        stop();
    }
}
