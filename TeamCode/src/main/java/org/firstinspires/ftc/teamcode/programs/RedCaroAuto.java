package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous
public class RedCaroAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(-28, -62, Math.toRadians(90));

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;
    private boolean active = false;

    @Override


    public void runOpMode() throws InterruptedException {

        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();


        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        TrajectorySequence sequence = robot.drive.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-58, -58), Math.toRadians(180))
                .addTemporalMarker(() -> robot.carousel.reverse())
                .waitSeconds(3)
                .addTemporalMarker(() -> active = true)
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-32, -26))
                .lineToLinearHeading(new Pose2d(-40, -24, Math.toRadians(240)))
                .splineToConstantHeading(new Vector2d(-44, -62), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-59, -62), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-32, -26, Math.toRadians(180)), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(1 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-58, -40), Math.toRadians(180))
                .build();
        robot.lift.liftReset();
        waitForStart();

        VisionPipeline.POS position = vision.detector.getPosition();


        robot.drive.drive.setPoseEstimate(startPose);
        robot.drive.drive.followTrajectorySequenceAsync(sequence);

        while (opModeIsActive()) {
            robot.drive.drive.update();
            robot.lift.update(robot);
            robot.lift.updatePID(robot.lift.target);


            if(active) {
                if (position == VisionPipeline.POS.LEFT) {
                    robot.lift.liftHigh();
                } else if (position == VisionPipeline.POS.CENTER) {
                    robot.lift.liftMid();
                } else {
                    robot.lift.liftMid();
                }
            } else {
                robot.lift.liftIntake();
            }
        }
    }
}
