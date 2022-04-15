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
//@Autonomous(name = "BlueOnlyCarousel")
//public class BlueOnlyCarousel extends LinearOpMode {
//    DcMotor lift1;
//
//    enum State {
//        DUCK,           // go to carousel
//        WAITDUCK,       // wait x seconds to spin
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
//        VisionPipeline.POS position = vision.detector.getPosition();
//
//        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
//        robot.lift.liftReset();
//        lift1 = hardwareMap.get(DcMotor.class, "lift1");
//
//        Pose2d startPose = new Pose2d(-28, 62, Math.toRadians(270));
//        robot.drive.drive.setPoseEstimate(startPose);
//
//        Trajectory Duck = robot.drive.drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(-44, 44), Math.toRadians(180))
//                .splineTo(new Vector2d(-58, 58), Math.toRadians(180))
//                .build();
//
//        Trajectory Park = robot.drive.drive.trajectoryBuilder(Duck.end())
//                .splineToLinearHeading(new Pose2d(-58, 40, Math.toRadians(0)), Math.toRadians(180))
//                .addDisplacementMarker(6, () -> {
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
//        robot.lift.liftReset();
//        robot.intake.intakeUp();
//
//        waitForStart();
//
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
//                        robot.carousel.on();
//                        currentState = State.WAITDUCK;
//                    }
//                    break;
//                case WAITDUCK:
//                    if (DuckTimer.seconds() >= DuckTime) {
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