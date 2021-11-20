package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drive implements Subsystem {
    Gamepad gamepad1;
    Gamepad gamepad2;
    SampleMecanumDrive drive;
    Servo rightOdo;
    Servo leftOdo;
    Servo centerOdo;

    public static double odoRDown;
    public static double odoLDown;
    public static double odoCDown;
    public static double odoRUp;
    public static double odoLUp;
    public static double odoCUp;

    public Drive(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        drive = new SampleMecanumDrive(hw);
//        rightOdo = hw.get(Servo.class, "rightOdo");
//        leftOdo = hw.get(Servo.class, "leftOdo");
//        centerOdo = hw.get(Servo.class, "centerOdo");
//        odoRetract();
    }

    @Override
    public void update(Robot robot) {
        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));
        drive.update();
    }

//    public void odoRetract() {
//        rightOdo.setPosition(odoRUp);
//        leftOdo.setPosition(odoLUp);
//        centerOdo.setPosition(odoCUp);
//    }
//
//    public void odoLower() {
//        rightOdo.setPosition(odoRDown);
//        leftOdo.setPosition(odoLDown);
//        centerOdo.setPosition(odoCDown);
//    }
}
