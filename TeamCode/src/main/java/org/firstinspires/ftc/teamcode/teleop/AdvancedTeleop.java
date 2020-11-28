package org.firstinspires.ftc.teamcode.teleop;

public class AdvancedTeleop extends AdvancedLib {

    @Override
    public void loop() {
        //Update to field centric
        robotCentricTrigMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        updateIntake();
        updateShooter();
        updateGrabber();
        updateTransfer();
    }
}
