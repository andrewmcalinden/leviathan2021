package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AdvancedTeleop", group = "18030")
public class AdvancedTeleop extends AdvancedLib {

    @Override
    public void loop() {
        double robotHeadingRad = gyro.getAngle() * (Math.PI / 180);
        //might need to negate left stick y because apparently that's a thing
        fieldCentricMecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeadingRad);
        //updateIntake(); //hold right bumper gamepad1
        updateShooter(); //toggle a and b gamepad1
        updateGrabber(); //right stick y gamepad2
//        updateTransfer(); //hold y for motor, tap x for servo gamepad1
    }
}
