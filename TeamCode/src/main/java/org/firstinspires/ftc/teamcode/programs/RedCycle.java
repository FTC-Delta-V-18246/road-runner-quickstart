package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

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
    BasicLift lift;
    State currentState = State.DONE;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        Pose2d startPose = new Pose2d(-32, -65, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        TrajectorySequence seq = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135))
                .addDisplacementMarker(16, () -> {
                    robot.v4b.deposit();
                })
                .splineToSplineHeading(new Pose2d(12, -63, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(48, -63, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(12, -63, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-62, -36, Math.toRadians(0)), Math.toRadians(180))
                .build();
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

        robot.drive.drive.followTrajectorySequenceAsync(seq);

        waitForStart();

        currentState = State.DUMP;
        robot.deposit.turretNeutral();

        switch (currentState) {
            case DUMPLOADED:
                if (position == VisionPipeline.POS.LEFT) {
                    lift.liftShared();
                } else if (position == VisionPipeline.POS.CENTER) {
                    lift.liftMid();
                } else {
                    lift.liftHigh();
                }
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.WAITDUMP;
                }

                break;
            case WAITDUMPLOADED:
                DumpTimer.reset();
                if (DumpTimer.seconds() >= DumpTime) {
                    robot.deposit.open();
                    currentState = State.INTAKE;
                }
                break;
            case INTAKE:
                DumpTimer.reset();
                if (DumpTimer.seconds() >= DumpTime) {
                    robot.v4b.intake();
                    robot.deposit.close();
                }
                ArmTimer.reset();
                if (ArmTimer.seconds() >= ArmTime) {
                    lift.liftIntake();
                }
                robot.intake.intakeDown();
                robot.intake.on();
                robot.drive.drive.followTrajectoryAsync(Intake);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.DUMP;
                }
                break;
            case DUMP:
                lift.liftHigh();
                ArmTimer.reset();
                if (ArmTimer.seconds() >= ArmTime) {
                    robot.deposit.open();
                }
                robot.drive.drive.followTrajectoryAsync(Dump);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.WAITDUMP;
                }
                break;
            case WAITDUMP:
                DumpTimer.reset();
                if (DumpTimer.seconds() >= DumpTime) {
                    robot.deposit.open();
                    currentState = State.PARK;
                }
                break;
            case PARK:
                robot.drive.drive.followTrajectory(Park);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.DONE;
                }
                break;
            case DONE:
                lift.liftIntake();
                break;
        }
    }
}

