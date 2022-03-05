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


@Autonomous(name = "RedDuck")
public class RedDuck extends LinearOpMode {
    DcMotor lift1;

    enum State {
        DUCK,           // go to carousel
        WAITDUCK,       // wait x seconds to spin
        DUMP,           // go to dump position (need if statement)
        WAITDUMP,       // wait x seconds to dump
        BACK,
        RESET,
        INTAKE,         // go to intake duck
        WAITINTAKE,
        DUMP2,          // go to dump high
        WAITDUMP2,      // wait to dump high
        PARK,           //go to park
        DONE            // nothing
    }

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;
    State currentState = State.DONE;

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

        Trajectory Duck = robot.drive.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-57, -57), Math.toRadians(180))
                .build();
        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Duck.end())
                .splineToConstantHeading(new Vector2d(-57.5, -30), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-32, -25), Math.toRadians(180))
                .addTemporalMarker(0.2, () -> {
                    robot.v4b.deposit();
                })
                .build();
        Trajectory Back = robot.drive.drive.trajectoryBuilder(Dump.end())
                //.lineToSplineHeading(new Pose2d(-57.5, -24, Math.toRadians(180)))
                .splineTo(new Vector2d(-40, -24), Math.toRadians(270))
                .addDisplacementMarker(() -> robot.v4b.intake())
                .splineTo(new Vector2d(-57.5, -24), Math.toRadians(270))
                .addDisplacementMarker( () -> {
                    robot.lift.liftIntake();
                    robot.deposit.closeDuck();
                })
        //.splineToSplineHeading(new Pose2d(-56, -48, Math.toRadians(270)), Math.toRadians(180))
                .splineTo(new Vector2d(-55, -59), Math.toRadians(270), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1.5, () -> {
                    robot.intake.on();
                })
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(Back.end())
                .lineToConstantHeading(new Vector2d(-44, -62), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory Dump2 = robot.drive.drive.trajectoryBuilder(Intake.end())
                .splineToLinearHeading(new Pose2d(-32, -25, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump2.end())
                .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(6, () -> {
                    robot.v4b.intake();
                    robot.deposit.close();
                })
                .build();
        double DuckTime = 3;
        ElapsedTime DuckTimer = new ElapsedTime();
        double DumpTime = 2;
        ElapsedTime DumpTimer = new ElapsedTime();
        double ResetTime = 2;
        ElapsedTime ResetTimer = new ElapsedTime();
        robot.lift.liftReset();
        robot.intake.intakeUp();

        waitForStart();
        VisionPipeline.POS position = vision.detector.getPosition();
        telemetry.addData("Position: ", position);

        robot.drive.drive.followTrajectory(Duck);
        currentState = State.DUCK;
        robot.lift.liftIntake();
        robot.v4b.intake();
        robot.deposit.close();

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.intake.intakeDown();
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();

            switch (currentState) {
                case DUCK:
                    if (!robot.drive.drive.isBusy()) {
                        DuckTimer.reset();
                        robot.carousel.reverse();
                        currentState = State.WAITDUCK;
                    }
                    break;
                case WAITDUCK:
                    if (DuckTimer.seconds() >= DuckTime) {
                        currentState = State.DUMP;
                        robot.drive.drive.followTrajectory(Dump);
                    }
                    if (position != VisionPipeline.POS.RIGHT) {
                        robot.lift.liftMid();
                    } else {
                        robot.lift.liftHigh();
                    }
                    if ((lift1.getCurrentPosition() < robot.lift.READY) && (position == VisionPipeline.POS.LEFT)) {
                            robot.lift.liftShared();
                        }
                    break;
                case DUMP:
                    robot.carousel.off();
                    robot.v4b.deposit();
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP:
                    if (DumpTimer.seconds() >= DumpTime) {
                        robot.deposit.open();
                        robot.lift.liftHigh();
                        robot.drive.drive.followTrajectory(Back);
                        currentState = State.BACK;
                    }
                    break;
                case BACK:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.RESET;
                        ResetTimer.reset();
                        robot.lift.liftIntake();
                    }
                    break;
                case RESET:
                    if (ResetTimer.seconds() >= ResetTime) {
                        currentState = State.INTAKE;
                        robot.drive.drive.followTrajectory(Intake);
                    }
                    break;
                case INTAKE:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITINTAKE;
                        DumpTimer.reset();
                    }
                    break;
                case WAITINTAKE:
                    robot.lift.liftHigh();
                    if (DumpTimer.seconds() >= DumpTime) {
                        currentState = State.DUMP2;
                        robot.drive.drive.followTrajectory(Dump2);
                    }
                    if (lift1.getCurrentPosition() < robot.lift.READY) {
                        robot.v4b.deposit();
                        robot.intake.off();
                    }
                    break;
                case DUMP2:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP2;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP2:
                    robot.deposit.open();
                    if (DumpTimer.seconds() >= DumpTime) {
                        currentState = State.PARK;
                        robot.drive.drive.followTrajectory(Park);
                    }
                    break;
                case PARK:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.DONE;
                    }
                    break;
                case DONE:
                    robot.lift.liftIntake();
                    robot.deposit.close();
                    robot.v4b.intake();
                    break;
            }
        }
    }
}