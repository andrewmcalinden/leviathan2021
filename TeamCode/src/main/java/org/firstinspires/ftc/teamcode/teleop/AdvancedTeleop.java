package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AdvancedTeleop", group = "18030")
public class AdvancedTeleop extends AdvancedLib {

    @Override
    public void loop(){
        //double robotHeadingRad = gyro.getAngle() * (Math.PI / 180);
        //fieldCentricMecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeadingRad);
        if (gamepad1.right_trigger != 0){
            robotCentricAdditiveMecanum(gamepad1.left_stick_x * .5, -gamepad1.left_stick_y * .5, gamepad1.right_stick_x * .5);
        }
        else{
            robotCentricAdditiveMecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        updateIntake(); //hold left bumper gamepad1
        updateShooter(); //toggle a gamepad1
        //updateGrabber(); //right stick y gamepad2, toggle a gamepad 2
        updateTranfer();
    }
}
