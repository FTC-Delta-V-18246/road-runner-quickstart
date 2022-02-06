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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous(name = "RedCycle")
public class RedCycle extends LinearOpMode {
    enum State {
        DUMPLOADED,           // go to dump position (need if statement)
        WAITDUMPLOADED,       // wait x seconds to dump
        INTAKE,         // go to intake duck
        DUMP,          // go to dump high
        WAITDUMP,      // wait to dump high
        PARK,           //go to park
        DONE            // nothing
    }

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;
    State currentState = State.DONE;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        Pose2d startPose = new Pose2d(15, -62, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        Trajectory DumpLoaded = robot.drive.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135))
                .addDisplacementMarker(16, () -> {
                    robot.v4b.deposit();
                })
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(DumpLoaded.end())
                .splineToSplineHeading(new Pose2d(8, -62, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(48, -62, Math.toRadians(0)), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Intake.end())
                .lineToSplineHeading(new Pose2d(8, -64, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135))
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump.end())
                .splineToSplineHeading(new Pose2d(8, -62, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(48, -62, Math.toRadians(0))).build();
        //timers
        double DumpTime = 1.5;
        ElapsedTime DumpTimer = new ElapsedTime();
        double ArmTime = 3;
        ElapsedTime ArmTimer = new ElapsedTime();
        //vision
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();
        VisionPipeline.POS position = vision.detector.getPosition();
        robot.lift.updatePID(0);

        waitForStart();

        robot.drive.drive.followTrajectory(DumpLoaded);
        currentState = State.DUMPLOADED;
        robot.lift.liftIntake();

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();

            switch (currentState) {
                case DUMPLOADED:
                    robot.deposit.close();
                    if (position == VisionPipeline.POS.LEFT) {
                        robot.lift.liftShared();
                    } else if (position == VisionPipeline.POS.CENTER) {
                        robot.lift.liftMid();
                    } else {
                        robot.lift.liftHigh();
                    }
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMPLOADED;
                        DumpTimer.reset();
                    }

                    break;
                case WAITDUMPLOADED:
                    if (DumpTimer.seconds() >= DumpTime) {
                        robot.deposit.open();
                        currentState = State.INTAKE;
                        robot.drive.drive.followTrajectory(Intake);
                        DumpTimer.reset();
                        ArmTimer.reset();
                    }
                    break;
                case INTAKE:
                    robot.intake.on();
                    if (DumpTimer.seconds() >= DumpTime) {
                        robot.v4b.intake();
                        robot.deposit.close();
                    }
                    if (ArmTimer.seconds() >= ArmTime) {
                        robot.lift.liftIntake();
                    }
                    robot.intake.intakeDown();
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.DUMP;
                        robot.lift.liftHold();
                        robot.drive.drive.followTrajectory(Dump);
                        ArmTimer.reset();
                    }
                    break;
                case DUMP:
                    robot.intake.reverse();

                    robot.lift.liftHigh();
                    if (ArmTimer.seconds() >= ArmTime) {
                        robot.deposit.open();
                    }
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP:
                    if (DumpTimer.seconds() >= DumpTime) {
                        robot.deposit.open();
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
                    break;
            }
            robot.update();
        }
    }
}