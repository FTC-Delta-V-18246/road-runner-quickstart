package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "autolong")
public class autoo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        waitForStart();
        Pose2d startPose = new Pose2d(-64, -42, Math.toRadians(180)); //64, 42
        robot.drive.drive.setPoseEstimate(startPose);

        Trajectory park = robot.drive.drive.trajectoryBuilder(startPose, true)
                .forward(67)
                .build();
        
        sleep(20000);
        robot.drive.drive.followTrajectory(park);
    }
}