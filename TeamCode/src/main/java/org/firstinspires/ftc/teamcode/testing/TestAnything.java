package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Grabber;

@TeleOp(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {


    Drivetrain dt;
    @Override
    public void runOpMode() throws InterruptedException {

        dt = new Drivetrain(this);
        waitForStart();

        while(!isStopRequested()) {
            //dt.turnHeading(90, .1, 0, .17, .14);
            break;
        }
        stop();
    }
}
