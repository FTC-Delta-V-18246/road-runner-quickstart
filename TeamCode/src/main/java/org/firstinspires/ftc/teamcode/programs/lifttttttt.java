//package org.firstinspires.ftc.teamcode.programs;
//
//import com.acmerobotics.roadrunner.drive.Drive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.BasicLift;
//import org.firstinspires.ftc.teamcode.util.Vision;
//import org.firstinspires.ftc.teamcode.util.VisionPipeline;
//import org.openftc.easyopencv.OpenCvCamera;
//
//
//@Autonomous(name = "lifttttttt")
//public class lifttttttt extends LinearOpMode {
//    enum State {
//        LIFT,
//        IDLE
//    }
//
//    State currentState = State.IDLE;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
//        robot.lift.liftReset();
//
//        waitForStart();
//
//        robot.v4b.intake();
//        currentState = State.LIFT;
//
//
//        while (opModeIsActive()) {
//            robot.deposit.turretNeutral();
//            robot.intake.intakeDown();
//
//            switch (currentState) {
//                case LIFT:
//                    robot.lift.liftHigh();
//                    robot.carousel.on();
//                    break;
//                case IDLE:
//                    robot.lift.liftHigh();
//                    break;
//            }
//            robot.lift.updatePID(robot.lift.target);
//
//        }
//    }
//}