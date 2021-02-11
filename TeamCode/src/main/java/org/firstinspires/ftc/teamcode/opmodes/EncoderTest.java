package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.math.Vector;

@Autonomous(name = "encoder test", group = "18030")
public class EncoderTest extends LinearOpMode {

    public Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this);
        waitForStart();

        //dt.movePIDFGyro(20, .8, 0, 0, .1 );
//        while (!isStopRequested()){
//            dt.goStraight(.6, 20);
//            //stop();
//        }

        dt.startMotors(1, 1);

    }
}
