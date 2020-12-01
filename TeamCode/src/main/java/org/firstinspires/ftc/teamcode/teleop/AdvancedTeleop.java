package org.firstinspires.ftc.teamcode.teleop;

public class AdvancedTeleop extends AdvancedLib {

    @Override
    public void loop() {
        double robotHeadingRad = gyro.getAngle() * (Math.PI / 180);
        //might need to negate left stick y because apparently that's a thing
        fieldCentricMecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeadingRad);
        updateIntake(); //hold right bumper
        updateShooter(); //toggle a and b
        updateGrabber(); //gamepad2 right stick y
        updateTransfer(); //hold y, tap x for servo
    }
}
