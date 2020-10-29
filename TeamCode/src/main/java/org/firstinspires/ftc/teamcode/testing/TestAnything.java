package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.math.Vector;

@TeleOp(name = "test anything", group = "18030")
public class TestAnything extends LinearOpMode {

    public Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {
        Vector test = new Vector(1, 0);
        //test = test.rotated(-180 * )
    }
}
