package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;

@Autonomous(name = "blue wobble mid", group = "18030")
public class BlueMidGoalWobble extends LinearOpMode {
    private Drivetrain dt;
    private Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);
        grabber = new Grabber(this);
        waitForStart();

        if (!isStopRequested()){
            grabber.goToNeck();
            grabber.closeGrabber();
            sleep(200);
            grabber.liftUp();

            dt.movePIDFGyro(-50, .5, 0, 0, .1);
        }


    }
}
