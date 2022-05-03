package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Drive implements Subsystem {
    Gamepad gamepad1;
    Gamepad gamepad2;
    public SampleMecanumDrive drive;
    Servo rightOdo;
    Servo leftOdo;
    Servo centerOdo;
    Servo leftBrake;
    Servo rightBrake;

    public static double odoRDown = 0.1;
    public static double odoLDown = 0.9;
    public static double odoCDown = 0.1;
    public static double odoRUp = 1;
    public static double odoLUp = 0;
    public static double odoCUp = 1;
    public static double rotatePower = 1.0;
    public static double brakeDown = 1.0;
    public static double brakeUp = 0.85;

    public Drive(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        drive = new SampleMecanumDrive(hw);
        rightOdo = hw.get(Servo.class, "rightOdo");
        leftOdo = hw.get(Servo.class, "leftOdo");
        centerOdo = hw.get(Servo.class, "centerOdo");
        rightBrake = hw.get(Servo.class, "rightBrake");
        leftBrake = hw.get(Servo.class, "leftBrake");

        odoLower();
        unBrake();
    }

    @Override
    public void update(Robot robot) {
        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x / rotatePower
        ));

        if (gamepad1.dpad_down) {
            Brake();
        }
        if (gamepad1.dpad_up) {
            unBrake();
        drive.update();        }

}

    public void odoRetract() {
        rightOdo.setPosition(odoRUp);
        leftOdo.setPosition(odoLUp);
        centerOdo.setPosition(odoCUp);
    }

    public void odoLower() {
        rightOdo.setPosition(odoRDown);
        leftOdo.setPosition(odoLDown);
        centerOdo.setPosition(odoCDown);
    }

    public void Brake() {
        rightBrake.setPosition(brakeDown);
        leftBrake.setPosition(1 - brakeDown);
    }

    public void unBrake() {
        rightBrake.setPosition(brakeUp);
        leftBrake.setPosition(1 - brakeUp);
    }
}
