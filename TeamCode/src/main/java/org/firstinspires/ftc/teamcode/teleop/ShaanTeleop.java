package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalHeading;

@TeleOp(name = "skurt skumar teleop", group = "18030")
public class ShaanTeleop extends AdvancedLib {

    double initialHeading = FinalHeading.finalHeading - 90;

    @Override
    public void loop(){
        double robotHeadingDeg = angleWrapDeg(gyro.getAngle() + initialHeading);
        double robotHeadingRad = robotHeadingDeg * (Math.PI / 180);
        if (gamepad1.right_trigger != 0){
            fieldCentricMecanum(gamepad1.left_stick_x * .5, -gamepad1.left_stick_y * .5, gamepad1.right_stick_x * .5, robotHeadingRad);
        }
        else{
            fieldCentricMecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeadingRad);
        }
        updateIntake();
        updateShooter();
        updateGrabber();
        updateTransfer();
        telemetry.addData("un altered heading", gyro.getAngle());
        telemetry.addData("initial heading", initialHeading);
        telemetry.addData("current heading", robotHeadingRad /** (180.0 / Math.PI)*/);
        telemetry.update();
    }

    double angleWrapDeg(double angle) {
        double correctAngle = angle;
        while (correctAngle > 180)
        {
            correctAngle -= 360;
        }
        while (correctAngle < -180)
        {
            correctAngle += 360;
        }
        return correctAngle;
    }
}
