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
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "BlueDuck")
public class BlueDuck extends LinearOpMode {
    enum State {
        DUCK,           // go to carousel
        WAITDUCK,       // wait x seconds to spin
        DUMP,           // go to dump position (need if statement)
        WAITDUMP,       // wait x seconds to dump
        INTAKE,         // go to intake duck
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
        //vision
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();
        VisionPipeline.POS position = vision.detector.getPosition();

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.lift.liftReset();
        Pose2d startPose = new Pose2d(-28, 62, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        Trajectory Duck = robot.drive.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-44, 44), Math.toRadians(180))
                .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                .build();
        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Duck.end())
                .lineToConstantHeading(new Vector2d(-32, 24))
                .addDisplacementMarker(4, () -> {
                    robot.v4b.deposit();
                    robot.carousel.off();
                    if (position == VisionPipeline.POS.LEFT) {
                        robot.lift.liftHold();
                    }
                })
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(Dump.end())
                .lineToSplineHeading(new Pose2d(-38,24, Math.toRadians(180)))
                .addTemporalMarker(0.5, () -> {
                    if (position == VisionPipeline.POS.LEFT) {
                        robot.lift.liftShared();
                    }                })
                .addTemporalMarker(1, () -> {
                    robot.v4b.intake();
                })
                .addTemporalMarker(2, () -> {
                    robot.lift.liftIntake();
                    robot.deposit.closeDuck();
                    robot.intake.on();
                })
                .splineToSplineHeading(new Pose2d(-44, 56, Math.toRadians(180)), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-56, 56, Math.toRadians(135)))
                .build();
        Trajectory Dump2 = robot.drive.drive.trajectoryBuilder(Intake.end())
                .lineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(180)))
                .addTemporalMarker(1, () -> {
                    robot.v4b.deposit();
                })
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump2.end())
                .addDisplacementMarker(6, () -> {
                    robot.v4b.intake();
                    robot.deposit.close();
                })
                .splineToLinearHeading(new Pose2d(-60, 42, Math.toRadians(0)), Math.toRadians(180))
                .build();
        //timers
        double DuckTime = 3;
        ElapsedTime DuckTimer = new ElapsedTime();
        double DumpTime = 1;
        ElapsedTime DumpTimer = new ElapsedTime();
        robot.lift.liftReset();

        waitForStart();

        robot.drive.drive.followTrajectory(Duck);
        currentState = State.DUCK;
        robot.lift.liftIntake();
        robot.v4b.intake();
        robot.deposit.close();

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.intake.intakeDown();

            switch (currentState) {
                case DUCK:
                    if (!robot.drive.drive.isBusy()) {
                        DuckTimer.reset();
                        robot.carousel.on();
                        currentState = State.WAITDUCK;
                    }
                    break;
                case WAITDUCK:
                    if (DuckTimer.seconds() >= DuckTime) {
                        currentState = State.DUMP;
                        robot.drive.drive.followTrajectory(Dump);
                    }
                    if (position == VisionPipeline.POS.LEFT) {
                        robot.lift.liftShared();
                    } else if (position == VisionPipeline.POS.CENTER) {
                        robot.lift.liftMid();
                    } else {
                        robot.lift.liftHigh();
                    }
                    break;
                case DUMP:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP;
                        DumpTimer.reset();
                        robot.deposit.open();
                    }
                    break;
                case WAITDUMP:
                    //v4b temporal marker, scroll up to traj
                    if (DumpTimer.seconds() >= DumpTime) {
                        currentState = State.INTAKE;
                        robot.drive.drive.followTrajectory(Intake);
                    }
                    break;
                case INTAKE:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.DUMP2;
                        robot.drive.drive.followTrajectory(Dump2);
                    }
                    break;
                case DUMP2:
                    robot.intake.off();
                    robot.lift.liftHigh();
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP2;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP2:
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
            robot.lift.updatePID(robot.lift.target);
        }
    }
}