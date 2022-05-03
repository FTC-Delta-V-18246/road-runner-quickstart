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

@Autonomous(name = "HoustonCycleRED")
public class CycleFsm extends LinearOpMode {
    DcMotor lift1;
    Vision vision;

    enum Deposit {
        HOME,
        HIGH,
        TRANSITION,
        RETRACT
    }

    enum State {
        PRELOAD,
        CYCLE,
        DONE
    }

    State currentState = State.DONE;
    Deposit deposit = Deposit.HOME;

    double time = 0.01;

    ElapsedTime depositTimer = new ElapsedTime();
    ElapsedTime DumpTimer = new ElapsedTime();
    ElapsedTime kickTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        lift1 = hardwareMap.get(DcMotor.class, "lift1");

        robot.intake.intakeUp();
        robot.v4b.receive();
        robot.deposit.close();
        robot.drive.odoLower();

        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(0));
        robot.drive.drive.setPoseEstimate(startPose);

        /*TrajectorySequence preload = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-12, -46))
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(20, -62), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(44, -62))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.intake.on(robot);
                    robot.intake.intakeRight();
                })
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                })
                .build();*/
        TrajectorySequence preloadHigh = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                }).lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    kick();
                })
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(45, -62),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 2
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(47, -62.5),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 3
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(48, -63),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 4
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(49, -63),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 5
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(50, -63.5),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(8, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                .lineToConstantHeading(new Vector2d(44, -63.5))
                .build();

        TrajectorySequence preloadMid = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                }).lineToConstantHeading(new Vector2d(-6, -62))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    kick();
                })
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(44, -62),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 2
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(45, -63),   robot.drive.drive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -63))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 3
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(46, -63),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -63))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 4
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(47, -63.5),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -63.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.off();
                    kick();
                })
                //cycle 5
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(48, -64),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -64))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intake.reverse();
                    kick();
                })
                .build();
        TrajectorySequence preloadLow = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                }).lineToConstantHeading(new Vector2d(-6, -48))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    kick();
                })
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(44, -62),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 2
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(45, -62.5),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -62.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 3
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(46, -63),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -63))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 4
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(47, -63.5),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -63.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    kick();
                })
                //cycle 5
                .addTemporalMarker(() -> {
                    robot.intake.autoRED();
                    robot.intake.intakeDown();
                })
                .lineToConstantHeading(new Vector2d(48, -64),   robot.drive.drive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        robot.drive.drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addDisplacementMarkerOffset(10, () -> {
                    robot.intake.reverse();
                    deposit();
                })
                .UNSTABLE_addDisplacementMarkerOffset(22, () -> {
                    robot.intake.off();
                    deposit();
                })
                .lineToConstantHeading(new Vector2d(-6, -64))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    kick();
                })
                .build();

        waitForStart();

        VisionPipeline.POS position = vision.detector.getPosition();

        robot.drive.drive.followTrajectorySequenceAsync(preloadHigh);

        while (opModeIsActive()) {

            /*switch (currentState) {
                case PRELOAD:
                    if (position == VisionPipeline.POS.LEFT) {
                        robot.v4b.shared();
                        robot.drive.drive.followTrajectorySequence(preload);
                    } else if (position == VisionPipeline.POS.CENTER) {
                        robot.v4b.deposit();
                        robot.drive.drive.followTrajectorySequence(preload);
                    } else {
                        if (depositTimer.milliseconds() > 500) {
                            robot.drive.drive.followTrajectorySequenceAsync(preloadHigh);
                        }
                        if (!robot.drive.drive.isBusy()) {
                            currentState = State.CYCLE;
                        }
                    }
                    break;
                case CYCLE:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.DONE;
                    }
                    break;
                case DONE:
                    break;
            }*/
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.addData("lift: ", lift1.getCurrentPosition());
            telemetry.addData("Position: ", position);
            telemetry.update();
            robot.intake.intakeRight();
            robot.intake.intakeDown();
            robot.drive.unBrake();

            switch (deposit) {
                case HOME:
                    robot.lift.liftHome();
                    robot.deposit.receive();
                    robot.v4b.intake(robot);
                    if ((robot.deposit.depositSensor <= 2.5)) {
                        robot.deposit.close();
                    }
                    break;
                case HIGH:
                    robot.deposit.close();
                    robot.v4b.deposit();
                    if (depositTimer.milliseconds() > 700) {
                        robot.lift.liftHigh();
                        robot.deposit.turretREDSHARED();
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
                        robot.intake.intakeDown();
                        deposit = Deposit.HOME;
                    }
                    break;
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