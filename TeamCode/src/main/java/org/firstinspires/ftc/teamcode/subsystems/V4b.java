package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class V4b implements Subsystem {

    Gamepad gamepad1;
    Gamepad gamepad2;
    Servo v4bLeft;
    Servo v4bRight;

    public static double deposit = 0.38; //maybe double check
    public static double intake = 0.92;

    public V4b(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        v4bLeft = hw.get(Servo.class, "v4bLeft");
        v4bRight = hw.get(Servo.class, "v4bRight");
        intake();
    }

    @Override
    public void update(Robot robot) {

    }


    public void intake() {
        v4bLeft.setPosition(intake);
        v4bRight.setPosition(1 - intake);
    }

    public void deposit() {
        v4bLeft.setPosition(deposit);
        v4bRight.setPosition(1 - deposit);
    }
}
