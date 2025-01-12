package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "velo test", group = "18030")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(1);
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}

