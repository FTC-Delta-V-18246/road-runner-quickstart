package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.subsystems.Drive;


@TeleOp
public class SampleTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        waitForStart();
        while (opModeIsActive()) {
            robot.drive.drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x / (robot.drive.rotatePower)
            ));
            robot.drive.odoRetract();
            robot.lift.ttelemetry(this);
            telemetry.update();
            robot.update();
        }
    }
}