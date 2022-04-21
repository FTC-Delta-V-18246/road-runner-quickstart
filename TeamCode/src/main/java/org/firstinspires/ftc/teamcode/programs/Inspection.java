package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.subsystems.BasicLift.READY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous(name = "Inspection")
public class Inspection extends LinearOpMode {
    DcMotor lift1;
    Vision vision;
    double time = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        lift1 = hardwareMap.get(DcMotor.class, "lift1");

        robot.intake.intakeUp();
        robot.v4b.receive();
        robot.deposit.close();
        robot.drive.odoLower();

        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(0));
        robot.drive.drive.setPoseEstimate(startPose);

        TrajectorySequence park = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(36, -46))
                .build();

        waitForStart();

        robot.drive.drive.followTrajectorySequenceAsync(park);

        while (opModeIsActive()) {

            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.update();
            robot.intake.intakeRight();

        }

    }
}