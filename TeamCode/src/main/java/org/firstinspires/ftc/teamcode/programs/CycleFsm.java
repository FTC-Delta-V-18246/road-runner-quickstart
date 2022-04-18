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


@Autonomous(name = "CycleFSM")
public class CycleFsm extends LinearOpMode {
    DcMotor lift1;
    Vision vision;

    enum Deposit {
        HOME,
        HIGH,
        TRANSITION,
        RETRACT
    }

    Deposit deposit = Deposit.HOME;
    ElapsedTime depositTimer;

    public static double ready = 100;
    public static double time = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, telemetry);
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
        robot.drive.odoLower();
        ElapsedTime DepositTImer = new ElapsedTime();
        ElapsedTime DumpTimer = new ElapsedTime();
        ElapsedTime kickTimer = new ElapsedTime();

        TrajectorySequence preload = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-12, -46))
                .addTemporalMarker(() -> {
                    kick();
                })
                .splineToConstantHeading(new Vector2d(20, -62), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                })
                .build();
        TrajectorySequence preloadHigh = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                }).lineToConstantHeading(new Vector2d(-12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(12, () -> {
                    robot.intake.off();
                })
                .build();

        TrajectorySequence cycle = robot.drive.drive.trajectorySequenceBuilder(preload.end())
                //cycle 1 score
                .lineToConstantHeading(new Vector2d(12, -62))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 2
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 3
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 4
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 5
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 6
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 7
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                })
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
                .lineToConstantHeading(new Vector2d(12, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                .build();

        waitForStart();

        VisionPipeline.POS position = vision.detector.getPosition();
        telemetry.addData("Position: ", position);
        depositTimer.reset();

        if (position == VisionPipeline.POS.LEFT) {
            robot.v4b.shared();
            robot.drive.drive.followTrajectorySequenceAsync(preload);
        } else if (position == VisionPipeline.POS.CENTER) {
            robot.v4b.deposit();
            robot.drive.drive.followTrajectorySequenceAsync(preload);
        } else {
            if (depositTimer.milliseconds() > 500) {
                robot.drive.drive.followTrajectorySequenceAsync(preloadHigh);
            }
        }
        robot.drive.drive.followTrajectorySequenceAsync(cycle);

        while (opModeIsActive()) {
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.addData("lift: ", lift1.getCurrentPosition());
            telemetry.update();

            switch (deposit) {
                case HOME:
                    robot.lift.liftHome();
                    robot.v4b.intake(robot);
                    if ((robot.deposit.depositSensor >= robot.deposit.distanceMax)) {
                        robot.deposit.close();
                        robot.intake.reverse();
                    }
                    break;
                case HIGH:
                    robot.v4b.deposit();
                    if (depositTimer.milliseconds() > 500) {
                        robot.lift.liftHigh();
                    }
                    break;
                case TRANSITION:
                    robot.deposit.kick();
                    DumpTimer.reset();
                    if (kickTimer.milliseconds() > robot.lift.kickTime) {
                        deposit = Deposit.RETRACT;
                    }
                    break;
                case RETRACT:
                    robot.deposit.turretNeutral();
                    robot.lift.liftIntake();
                    robot.drive.rotatePower = 1.0;
                    if (lift1.getCurrentPosition() < READY && DumpTimer.milliseconds() > 500) {
                        robot.v4b.intake(robot);
                        deposit = Deposit.HOME;
                    }
            }
        }
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