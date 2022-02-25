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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
import org.firstinspires.ftc.teamcode.util.Vision;
import org.firstinspires.ftc.teamcode.util.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous(name = "StatesRedCycle")
public class StatesRedCycle extends LinearOpMode {
    DcMotor lift1;

    enum State {
        DUMPLOADED,           // go to dump position (need if statement)
        WAITDUMPLOADED,       // wait x seconds to dump
        INTAKE,         // go to intake duck
        DUMP,          // go to dump high
        WAITDUMP,      // wait to dump high
        PARK,           //go to park
        DONE            // nothing
    }

    OpenCvCamera camera;
    Vision vision;
    VisionPipeline position;
    State currentState = State.DONE;
    //private static int cycleCount = 3;

    @Override
    public void runOpMode() throws InterruptedException {

        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.lift.liftReset();
        lift1 = hardwareMap.get(DcMotor.class, "lift1");

        Pose2d startPose = new Pose2d(15, -62, Math.toRadians(90));
        robot.drive.drive.setPoseEstimate(startPose);

        Trajectory DumpLoaded = robot.drive.drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(6, -42, Math.toRadians(315)), Math.toRadians(135))
                .build();
        Trajectory Intake = robot.drive.drive.trajectoryBuilder(DumpLoaded.end())
                .splineToSplineHeading(new Pose2d(6, -62, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(48, -62, Math.toRadians(0)), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Intake.end())
                .lineToSplineHeading(new Pose2d(8, -62.5, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(6, -42, Math.toRadians(315)), Math.toRadians(135))
                .build();
        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump.end())
                .splineToSplineHeading(new Pose2d(6, -63, Math.toRadians(0)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(48, -62, Math.toRadians(0)))
                .build();

        //timers
        double DumpTime = 1.5;
        ElapsedTime DumpTimer = new ElapsedTime();
        double ArmTime = 3;
        ElapsedTime ArmTimer = new ElapsedTime();
        //vision
        vision = new Vision(hardwareMap, telemetry);
        vision.setPipeline();
        vision.startStreaming();
        robot.lift.liftReset();

        waitForStart();

        VisionPipeline.POS position = vision.detector.getPosition();
        robot.drive.drive.followTrajectory(DumpLoaded);
        currentState = State.DUMPLOADED;
        robot.lift.liftIntake();
        robot.v4b.intake();
        robot.deposit.close();

        while (opModeIsActive()) {
            robot.deposit.turretNeutral();
            robot.intake.intakeDown();

            switch (currentState) {
                case DUMPLOADED:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMPLOADED;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMPLOADED:
                    if (DumpTimer.seconds() >= DumpTime) {
                        currentState = State.INTAKE;
                        robot.drive.drive.followTrajectory(Intake);
                    }
                    break;
                case INTAKE:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.DUMP;
                        robot.drive.drive.followTrajectory(Dump);
                    }
                    break;
                case DUMP:
                    if (!robot.drive.drive.isBusy()) {
                        currentState = State.WAITDUMP;
                        DumpTimer.reset();
                    }
                    break;
                case WAITDUMP:
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
                    break;
            }
            robot.lift.updatePID(robot.lift.target);
        }
    }
}