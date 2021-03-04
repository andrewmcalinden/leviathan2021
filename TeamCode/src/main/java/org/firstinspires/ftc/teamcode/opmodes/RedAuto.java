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
import org.firstinspires.ftc.teamcode.FinalHeading;

@Autonomous(name = "red auto", group = "18030")
public class RedAuto extends LinearOpMode {
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
            dt.movePIDFGyro(63, .8, 0, 0, .14);
            dt.strafeGyro(.5, 20);
            dt.turnHeading(15, .3, 0, 0, .14);
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
            sleep(300);
            mtrShooter.setPower(0);
            transfer.setPower(0);

            switch (numRings){
                case 0:
                    dt.movePIDFGyro(20, .4, 0, 0, .14);
                    dt.turnHeading(90, .6, 0, 0, .14);
                    dt.movePIDFGyro(-15, .3, 0, 0, .14);
                    grabber.deployWobble();
                    dt.movePIDFGyro(10, .3, 0, 0, .14);
                    break;
                case 1:
                    dt.turnHeading(180, .8, 0, 0, .14);
                    dt.movePIDFGyro(-37, .6, 0, 0, .14);
                    dt.strafeGyro(.3, -5);
                    grabber.deployWobble();
                    dt.movePIDFGyro(20, .6, 0, 0, .14);
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
            FinalHeading.finalHeading = dt.gyro.getAngle();
            break;
        }
        stop();
    }
}
