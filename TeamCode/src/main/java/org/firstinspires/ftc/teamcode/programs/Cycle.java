package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;


@Autonomous(name = "Cycle")
public class Cycle extends LinearOpMode {
    DcMotor lift1;

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.lift.liftReset();
        lift1 = hardwareMap.get(DcMotor.class, "lift1");

        Pose2d startPose = new Pose2d(-28, -62, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        double DuckTime = 3;
        ElapsedTime DuckTimer = new ElapsedTime();
        double DumpTime = 2;
        ElapsedTime DumpTimer = new ElapsedTime();
        double ResetTime = 1;
        ElapsedTime ResetTimer = new ElapsedTime();
        robot.lift.liftReset();
        robot.intake.intakeUp();

        waitForStart();
        VisionPipeline.POS position = vision.detector.getPosition();
        telemetry.addData("Position: ", position);

        robot.lift.liftIntake();
        robot.v4b.intake(robot);
        robot.deposit.close();

        robot.drive.drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-4, -48, Math.toRadians(315)), Math.toRadians(90))
                .addTemporalMarker(0.5, () -> {robot.lift.liftHigh();})
                .addDisplacementMarker(() -> {
                    robot.deposit.open();
                })
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(36, -61, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.v4b.intake(robot);
                    robot.deposit.close();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.lift.liftIntake();
                    robot.intake.on();
                })
                .splineToLinearHeading(new Pose2d(-4, -48, Math.toRadians(315)), Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.lift.liftHigh();
                })
                //repeat
                .build();

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.intake.intakeDown();
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();

        }
    }
}