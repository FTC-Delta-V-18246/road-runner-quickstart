package org.firstinspires.ftc.teamcode.programs;

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


@Autonomous(name = "CycleFSM")
public class CycleFsm extends LinearOpMode {
    DcMotor lift1;

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;

    enum Deposit {
        HOME,
        HIGH,
        TRANSITION
    }

    Deposit deposit = Deposit.HOME;
    ElapsedTime depositTimer;

    public static double ready = 100;
    public static double time = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.lift.liftReset();
        lift1 = hardwareMap.get(DcMotor.class, "lift1");

        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        robot.intake.intakeUp();
        robot.v4b.receive();
        robot.deposit.close();
        depositTimer = new ElapsedTime();

        TrajectorySequence cycle = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                })
                .splineToLinearHeading(new Pose2d(4, -38, Math.toRadians(315)), Math.toRadians(315-180))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(.200)
                //cycle 1

                .splineTo(new Vector2d(14, -63), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .splineTo(new Vector2d(43, -63), Math.toRadians(0), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.62 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineTo(new Vector2d(14, -63))
                .splineTo(new Vector2d(4, -38), Math.toRadians(130))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
                //cycle 2

                .splineTo(new Vector2d(14, -64), Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .splineTo(new Vector2d(45, -64), Math.toRadians(0), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.62 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineTo(new Vector2d(14, -64))
                .splineTo(new Vector2d(4, -38), Math.toRadians(125))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
                //cycle 3

                .splineTo(new Vector2d(14, -65), Math.toRadians(-3))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .splineTo(new Vector2d(47, -65), Math.toRadians(-3), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.62 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineTo(new Vector2d(14, -65))
                .splineTo(new Vector2d(4, -38), Math.toRadians(120))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
                //cycle 4
                .splineTo(new Vector2d(14, -66), Math.toRadians(5))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .splineTo(new Vector2d(47, -66), Math.toRadians(-5), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.62 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineTo(new Vector2d(14, -66))
                .splineTo(new Vector2d(2, -36), Math.toRadians(115))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
                .splineTo(new Vector2d(14, -66.5), Math.toRadians(-15))
                .splineTo(new Vector2d(46, -67), Math.toRadians(-15))

                //repeat
                .build();

        waitForStart();

        robot.intake.intakeDown();
        robot.drive.drive.followTrajectorySequenceAsync(cycle);

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.addData("lift: ", lift1.getCurrentPosition());
            telemetry.update();

            switch(deposit) {
                case HOME:
                    robot.lift.liftIntake();
                    robot.v4b.intake(robot);
                    if ((robot.deposit.distanceMax >= robot.deposit.depositSensor && robot.deposit.depositSensor >= robot.deposit.distanceMin)) {
                        robot.deposit.close();
                        robot.v4b.receive();
                        robot.intake.reverse();
                    }
                    break;
                case HIGH:
                    if(depositTimer.milliseconds() > 100) {
                        robot.lift.liftHigh();
                    }
                    if(Math.abs(lift1.getCurrentPosition()) >= ready) {
                        robot.v4b.deposit();
                    } else {
                        robot.v4b.receive();
                    }
                    break;
                case TRANSITION:
                    robot.deposit.kick();
                    if(depositTimer.milliseconds() > 200) {
                        robot.v4b.receive();
                    }
                    if(depositTimer.milliseconds() > (200 + 1100)) {
                        deposit = Deposit.HOME;
                    }
                    break;
            }        }
    }
    public void deposit() {
        depositTimer.reset();
        deposit = Deposit.HIGH;
    }

    public void kick() {
        depositTimer.reset();
        deposit = Deposit.TRANSITION;
    }
}