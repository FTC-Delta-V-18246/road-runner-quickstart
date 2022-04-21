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

@Autonomous(name = "HoustonBlueDuck")
public class HoustonBLueDuck extends LinearOpMode {
    DcMotor lift1;
    Vision vision;

    enum Deposit {
        HOME,
        HIGH,
        TRANSITION,
        RETRACT
    }

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

        Pose2d startPose = new Pose2d(-36, 62, Math.toRadians(180));
        robot.drive.drive.setPoseEstimate(startPose);

        TrajectorySequence preload = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    deposit();
                 })
                .splineToLinearHeading(new Pose2d(-42, 56, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    kick();
                    kickTimer.reset();
                })
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-66, 48, Math.toRadians(180)), Math.toRadians(180))
                .build();

        waitForStart();

        VisionPipeline.POS position = vision.detector.getPosition();

        robot.drive.drive.followTrajectorySequenceAsync(preload);

        while (opModeIsActive()) {

            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.addData("lift: ", lift1.getCurrentPosition());
            telemetry.addData("Position: ", position);
            telemetry.update();
            robot.intake.intakeRight();

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
                    if (depositTimer.milliseconds() > 700) {
                        robot.lift.liftHigh();
                    }
                    break;
                case TRANSITION:
                    robot.deposit.kick();
                    DumpTimer.reset();
                    if (kickTimer.milliseconds() > 1000) {
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