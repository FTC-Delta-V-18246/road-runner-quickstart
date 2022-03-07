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
                .lineToLinearHeading(new Pose2d(-25.5, -39.5, Math.toRadians(225)))
                .addTemporalMarker(0.2, () -> {
                    robot.v4b.deposit();
                })
                .build();
        Trajectory Back = robot.drive.drive.trajectoryBuilder(Dump.end())
                .splineToConstantHeading(new Vector2d(-32, -52), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    if (lift1.getCurrentPosition() < robot.lift.READY) {
                        robot.v4b.intake();}
                })
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(Back.end())
                .splineToConstantHeading(new Vector2d(-32, -61), Math.toRadians(180))
                .addDisplacementMarker( () -> {
                    robot.lift.liftIntake();
                    robot.deposit.closeDuck();
                })
                .lineToConstantHeading(new Vector2d(-54, -61), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> {
                    robot.intake.autoOn();
                })
                .build();
        Trajectory Dump2 = robot.drive.drive.trajectoryBuilder(Intake.end())
                .splineToConstantHeading(new Vector2d(-26, -40), Math.toRadians(180))
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump2.end())
                .splineToLinearHeading(new Pose2d(-59, -38, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(6, () -> {
                    robot.v4b.intake();
                    robot.deposit.close();
                })
                .build();
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
                    if (position != VisionPipeline.POS.CENTER) {
                        robot.lift.liftHigh();
                    } else {
                        robot.lift.liftMid();
                    }
                    break;
                case DUMP:
                    robot.carousel.off();
                    robot.v4b.deposit();
                    /*if ((lift1.getCurrentPosition() < robot.lift.READY) && (position == VisionPipeline.POS.LEFT)) {
                        robot.lift.liftShared(); //THIS BRINGS DOWN TO SHARED
                    }*/
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP:
                    if (DumpTimer.seconds() >= DumpTime) {
                        robot.deposit.open();
                        robot.drive.drive.followTrajectory(Back);
                        currentState = State.BACK;
                    }
                    break;
                case BACK:
                    /*if (lift1.getCurrentPosition() < robot.lift.READY) {
                        robot.v4b.intake();}*/
                    robot.lift.liftMid();
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.RESET;
                        ResetTimer.reset();
                    }
                    break;
                case RESET:
                    /*if (lift1.getCurrentPosition() < robot.lift.READY) {
                        robot.v4b.intake();} //REDUNDANT
                        */
                    if (lift1.getCurrentPosition() < robot.lift.READY) {
                        robot.lift.liftIntake();
                    }
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