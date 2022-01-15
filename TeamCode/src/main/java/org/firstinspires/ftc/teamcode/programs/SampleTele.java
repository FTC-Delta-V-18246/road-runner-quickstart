package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

@TeleOp
public class SampleTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            robot.carousel.updateCaro(-0.6);
            robot.deposit.turretNeutral();
            robot.deposit.close();
            robot.v4b.intake();

            if (gamepad1.dpad_left) {
                robot.v4b.deposit();
            }

            robot.update();
        }
    }
}
