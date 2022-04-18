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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;


@Autonomous(name = "Cycle")
public class Cycle extends LinearOpMode {
    DcMotor lift1;

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;

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

        //robot.intake.intakeUp();
        robot.v4b.receive();
        robot.deposit.close();

        TrajectorySequence cycle = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> robot.lift.liftHigh())
                .splineToSplineHeading(new Pose2d(2, -36, (Math.toRadians(315))), Math.toRadians(90))
                .addDisplacementMarker(6, () -> {
                    robot.v4b.deposit();
                })
                .addDisplacementMarker(() -> {
                    robot.deposit.kick();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.v4b.receive();
                    robot.deposit.receive();
                })
                .splineTo(new Vector2d(14, -63), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.lift.liftIntake();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    robot.v4b.intake(robot);
                    robot.intake.on(robot);
                })
                .splineTo(new Vector2d(40, -63), Math.toRadians(0), new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    robot.deposit.close();
                    robot.intake.reverse();
                })
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {
                    robot.intake.off();
                    robot.v4b.receive();
                })
                .lineTo(new Vector2d(14, -63))
                .UNSTABLE_addDisplacementMarkerOffset(6, () -> {
                    robot.lift.liftHigh();
                })
                .UNSTABLE_addDisplacementMarkerOffset(4, () -> {
                    robot.v4b.deposit();
                })
                .splineTo(new Vector2d(2, -36), Math.toRadians(135))
                .waitSeconds(0.25)
                .addDisplacementMarker(() ->
                        robot.deposit.kick())
                .splineTo(new Vector2d(2, -36), Math.toRadians(135))

                .build();

        waitForStart();

        //robot.intake.intakeDown();
        robot.drive.drive.followTrajectorySequenceAsync(cycle);

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.lift.updatePID(robot.lift.target);
            lift1.getCurrentPosition();
            robot.drive.drive.update();
            telemetry.addData("lift: ", lift1.getCurrentPosition());
            telemetry.update();
        }
    }
}