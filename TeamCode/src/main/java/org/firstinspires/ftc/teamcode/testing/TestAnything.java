package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.math.Vector;

@Autonomous(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    Drivetrain dt;
    Grabber grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);
        grabber = new Grabber(this);

        waitForStart();
        while (!isStopRequested()){
            dt.strafeGyro(.5, 67);
            sleep(500);
            dt.strafeGyro(.5, -67);
            break;
        }
        stop();
    }
}

//make goToPos method for grabber
//fix spaghet
//add pid to strafe gyro
