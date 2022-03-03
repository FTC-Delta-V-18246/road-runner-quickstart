package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import static org.firstinspires.ftc.teamcode.subsystems.Carousel.carouselpower;

@TeleOp
public class SampleTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            robot.intake.intakeDown();
            robot.lift.ttelemetry(this);
            telemetry.update();
            robot.carousel.updateCaro(0.25);

            robot.update();
        }
    }
}
