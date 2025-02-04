//package org.firstinspires.ftc.teamcode.programs;
//
//import com.acmerobotics.roadrunner.drive.Drive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
//import org.firstinspires.ftc.teamcode.util.Vision;
//import org.firstinspires.ftc.teamcode.util.VisionPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//
//import java.util.Arrays;
//
//@Autonomous(name = "StatesRedDuck")
//public class StatesRedDuck extends LinearOpMode {
//    DcMotor lift1;
//
//    enum State {
//        DUCK,           // go to carousel
//        WAITDUCK,       // wait x seconds to spin
//        DUMP,           // go to dump position (need if statement)
//        WAITDUMP,       // wait x seconds to dump
//        BACK,
//        RESET,
//        INTAKE,         // go to intake duck
//        WAITINTAKE,
//        DUMP2,          // go to dump high
//        WAITDUMP2,      // wait to dump high
//        PARK,           //go to park
//        DONE            // nothing
//    }
//
//    OpenCvCamera camera;
//    Vision vision;
//    VisionPipeline position;
//    State currentState = State.DONE;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //vision
//        vision = new Vision(hardwareMap, telemetry);
//        vision.setPipeline();
//        vision.startStreaming();
//
//        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
//        robot.lift.liftReset();
//        lift1 = hardwareMap.get(DcMotor.class, "lift1");
//
//        Pose2d startPose = new Pose2d(-28, -62, Math.toRadians(90));
//        robot.drive.drive.setPoseEstimate(startPose);
//
//        Trajectory Duck = robot.drive.drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(-58, -58), Math.toRadians(180))
//                .build();
//        Trajectory Dump = robot.drive.drive.trajectoryBuilder(Duck.end())
//                .lineToConstantHeading(new Vector2d(-32, -26))
//                .build();
//        Trajectory Back = robot.drive.drive.trajectoryBuilder(Dump.end())
//                .lineToLinearHeading(new Pose2d(-40, -24, Math.toRadians(240)))
//                .addDisplacementMarker(()->
//                        robot.intake.on()
//                )
//                .build();
//        Trajectory Intake = robot.drive.drive.trajectoryBuilder(Back.end())
//                .splineToConstantHeading(new Vector2d(-44, -62), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(-59, -62), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(0.5 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory Dump2 = robot.drive.drive.trajectoryBuilder(Intake.end())
//                .lineToLinearHeading(new Pose2d(-32, -26, Math.toRadians(180)), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(1 * DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        Trajectory Park = robot.drive.drive.trajectoryBuilder(Dump2.end())
//                //.splineToConstantHeading(new Vector2d(-58, -40), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(0)), Math.toRadians(180))
//                .addTemporalMarker(0.75, () -> {
//                    robot.v4b.intake();
//                    robot.deposit.close();
//                })
//                .build();
//        double DuckTime = 3;
//        ElapsedTime DuckTimer = new ElapsedTime();
//        double DumpTime = 1.5;
//        ElapsedTime DumpTimer = new ElapsedTime();
//        double ResetTime = 1;
//        ElapsedTime ResetTimer = new ElapsedTime();
//
//        robot.lift.liftReset();
//
//        waitForStart();
//
//        VisionPipeline.POS position = vision.detector.getPosition();
//        robot.drive.drive.followTrajectory(Duck);
//        currentState = State.DUCK;
//        robot.lift.liftIntake();
//        robot.v4b.intake();
//        robot.deposit.close();
//
//        while (opModeIsActive()) {
//            robot.deposit.turretNeutral();
//            robot.intake.intakeDown();
//            robot.lift.updatePID(robot.lift.target);
//            lift1.getCurrentPosition();
//
//            switch (currentState) {
//                case DUCK:
//                    if (!robot.drive.drive.isBusy()) {
//                        DuckTimer.reset();
//                        robot.carousel.reverse();
//                        currentState = State.WAITDUCK;
//                        DumpTimer.reset();
//                    }
//                    break;
//                case WAITDUCK:
//                    if (DuckTimer.seconds() >= DuckTime) {
//                        currentState = State.DUMP;
//                        robot.drive.drive.followTrajectory(Dump);
//                        robot.carousel.off();
//                        ResetTimer.reset();
//                    }
//                    if (position == VisionPipeline.POS.LEFT) {
//                        robot.lift.liftHigh();
//                    } else if (position == VisionPipeline.POS.CENTER) {
//                        robot.lift.liftMid();
//                    } else {
//                        robot.lift.liftMid();
//                    }
//                    if (lift1.getCurrentPosition() < robot.lift.READY) {
//                        robot.v4b.deposit();
//                    }
//                    break;
//                case DUMP:
//                    if (position == VisionPipeline.POS.LEFT && ResetTimer.seconds() >= 1) {
//                        robot.lift.liftShared();
//                    }
//                    if (!robot.drive.drive.isBusy()) {
//                        currentState = State.WAITDUMP;
//                        DumpTimer.reset();
//                        robot.deposit.open();
//                    }
//                    break;
//                case WAITDUMP:
//                    if (DumpTimer.seconds() >= DumpTime) {
//                        robot.drive.drive.followTrajectory(Back);
//                        currentState = State.BACK;
//                    }
//                    break;
//                case BACK:
//                    robot.lift.liftMid();
//                    if (!robot.drive.drive.isBusy()) {
//                        currentState = State.RESET;
//                        robot.v4b.intake();
//                        robot.deposit.closeDuck();
//                    }
//                    break;
//                case RESET:
//                    if (ResetTimer.seconds() >= ResetTime) {
//                        robot.lift.liftIntake();
//                        currentState = State.INTAKE;
//                        robot.drive.drive.followTrajectory(Intake);
//                    }
//                    break;
//                case INTAKE:
//                    if (!robot.drive.drive.isBusy()) {
//                        currentState = State.WAITINTAKE;
//                        DumpTimer.reset();
//                    }
//                    break;
//                case WAITINTAKE:
//                    robot.lift.liftHigh();
//                    robot.intake.off();
//                    if (lift1.getCurrentPosition() < robot.lift.READY) {
//                        robot.v4b.deposit();
//                    }
//                    if (DumpTimer.seconds() >= DumpTime) {
//                        currentState = State.DUMP2;
//                        robot.drive.drive.followTrajectory(Dump2);
//                    }
//                    break;
//                case DUMP2:
//                    if (lift1.getCurrentPosition() < robot.lift.READY) {
//                        robot.v4b.deposit();
//                    }
//                    if (!robot.drive.drive.isBusy()) {
//                        currentState = State.WAITDUMP2;
//                        DumpTimer.reset();
//                    }
//                    break;
//                case WAITDUMP2:
//                    robot.deposit.open();
//                    if (DumpTimer.seconds() >= DumpTime) {
//                        currentState = State.PARK;
//                        robot.drive.drive.followTrajectory(Park);
//                    }
//                    break;
//                case PARK:
//                    if (!robot.drive.drive.isBusy()) {
//                        currentState = State.DONE;
//                    }
//                    break;
//                case DONE:
//                    robot.lift.liftIntake();
//                    break;
//            }
//        }
//    }
//}