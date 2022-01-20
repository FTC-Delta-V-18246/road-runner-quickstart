package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "RedDuck")
public class RedDuck extends LinearOpMode {
    enum State {
        DUCK,           // go to carousel
        WAITDUCK,       // wait x seconds to spin
        DUMP,           // go to dump position (need if statement)
        WAITDUMP,       // wait x seconds to dump
        INTAKE,         // go to intake duck
        DUMP2,          // go to dump high
        WAITDUMP2,      // wait to dump high
        PARK,           //go to park
        IDLE            // nothing
    }

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        Pose2d startPose = new Pose2d(-32, -65, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        Trajectory Duck = robot.drive.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-48, -44, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56, -48, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Duck.end())
                .lineToConstantHeading(new Vector2d(-32, -24))
                .addDisplacementMarker(88, () -> {
                })
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(Dump.end())
                .splineToLinearHeading(new Pose2d(-40, -56, Math.toRadians(240)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -56))
                .build();
        Trajectory Dump2 = robot.drive.drive.trajectoryBuilder(Intake.end())
                .lineToLinearHeading(new Pose2d(-32, -24, Math.toRadians(180)))
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump2.end())
                .splineToLinearHeading(new Pose2d(-62, -36, Math.toRadians(0)), Math.toRadians(180))
                .build();

        double DuckTime = 3;
        ElapsedTime DuckTimer = new ElapsedTime();
        double DumpTime = 1.5;
        ElapsedTime DumpTimer = new ElapsedTime();

        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        waitForStart();

        currentState = State.DUCK;

        switch (currentState) {
            case DUCK:
                robot.drive.drive.followTrajectoryAsync(Duck);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.WAITDUCK;
                }
                break;
            case WAITDUCK:
                robot.carousel.on();
                DuckTimer.reset();
                if (DuckTimer.seconds() >= DuckTime) {
                    currentState = State.DUMP;
                }
                break;
            case DUMP:
                robot.drive.drive.followTrajectory(Dump);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.WAITDUMP;
                }
                break;
            case WAITDUMP:
                robot.deposit.open();
                DumpTimer.reset();
                if (DumpTimer.seconds() >= DumpTime) {
                    currentState = State.INTAKE;
                }
                break;
            case INTAKE:
                robot.intake.on();
                robot.drive.drive.followTrajectoryAsync(Intake);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.DUMP2;
                }
                break;
            case DUMP2:
                robot.drive.drive.followTrajectoryAsync(Dump2);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.WAITDUMP2;
                }
                break;
            case WAITDUMP2:
                robot.deposit.open();
                DumpTimer.reset();
                if (DumpTimer.seconds() >= DumpTime) {
                    currentState = State.IDLE;
                }
                break;
            case PARK:
                robot.drive.drive.followTrajectory(Park);
                if (!robot.drive.drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                break;
        }
    }
}