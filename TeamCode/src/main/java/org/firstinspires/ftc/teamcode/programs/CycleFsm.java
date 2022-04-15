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
                .lineToConstantHeading(new Vector2d(-12, -38))
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(.200)
                //cycle 1
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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
                .waitSeconds(time)
                .addTemporalMarker(() -> {
                    kick();
                })
                .waitSeconds(0.200)
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