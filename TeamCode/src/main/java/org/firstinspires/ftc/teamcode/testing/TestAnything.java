package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.math.Vector;

@Autonomous(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);

        waitForStart();
        while (!isStopRequested()){
//            dt.turnHeading(-90, .6, 0, 0, .14);
//            dt.turnHeading(70, .6, 0, 0, .14);
//            telemetry.addLine("done with turns");
//            telemetry.update();
//            sleep(1000);

            //dt.movePIDFGyro(20, .6, 0, 0, .14);
            telemetry.addLine("done with first straight");
            telemetry.update();
            sleep(1000);

            telemetry.clear();
            telemetry.update();

            dt.movePIDFGyro(-80, .6, 0, 0, .14);
            telemetry.addLine("done with second straight");
            telemetry.update();
            sleep(1000);

            telemetry.clear();
            telemetry.update();


            dt.strafeGyro(.5, 20);
            telemetry.addLine("done with first strafe");
            telemetry.update();
            sleep(1000);

            dt.strafeGyro(.5, -20);
            telemetry.addLine("done with second straight");
            telemetry.update();
            sleep(1000);

            break;
        }
        stop();
    }
}
