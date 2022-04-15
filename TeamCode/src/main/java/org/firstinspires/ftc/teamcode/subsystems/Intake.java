package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Intake implements Subsystem {
    public static Servo droptakeRight;
    public static Servo droptakeLeft;
    Gamepad gamepad1;
    Gamepad gamepad2;
    DcMotor intakeLeft;
    DcMotor intakeRight;

    public static double dropTakeDown = 0.31;
    public static double dropTakeUp = 0;
    public static double intakePower = 1.0;
    public static double autoPower = 0.805;


    public Intake(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        intakeLeft = hw.get(DcMotor.class, "intakeLeft");
        intakeRight = hw.get(DcMotor.class, "intakeRight");
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        droptakeLeft = hw.get(Servo.class, "droptakeL");
        droptakeRight = hw.get(Servo.class, "droptakeR");
    }

    @Override
    public void update(Robot robot) {
        intakeRight.setPower(intakePower * (gamepad1.left_trigger - gamepad1.right_trigger));
        intakeLeft.setPower(intakePower * (gamepad1.right_trigger - gamepad1.left_trigger));
    }

    public void intakeUp() {
        droptakeLeft.setPosition(1);
        droptakeRight.setPosition(0);
    }

    public void on(Robot robot) {
        intakeLeft.setPower(-intakePower);
        intakeRight.setPower(intakePower);
    }
    public void autoOn() {
        intakeLeft.setPower(-autoPower);
        intakeRight.setPower(-autoPower);
    }
    public void off() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public void reverse() {
        intakeLeft.setPower(0.6);
        intakeRight.setPower(-0.6);
    }
    public void intakeDown() {
        droptakeRight.setPosition(dropTakeDown);
        droptakeLeft.setPosition(1 - dropTakeDown);
    }
}