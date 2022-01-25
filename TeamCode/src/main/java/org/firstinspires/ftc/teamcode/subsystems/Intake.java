package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot;
import com.acmerobotics.dashboard.config.Config;


public class Intake implements Subsystem {
    public static Servo droptakeRight;
    public static Servo droptakeLeft;
    Gamepad gamepad1;
    Gamepad gamepad2;
    DcMotor intakeMotor;


    public static double dropTakeDown = 0.36;
    public static double dropTakeUp = 0;


    public Intake(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        droptakeLeft = hw.get(Servo.class, "droptakeL");
        droptakeRight = hw.get(Servo.class, "droptakeR");

    }

    @Override
    public void update(Robot robot) {
        intakeMotor.setPower(1.0 * (gamepad1.right_trigger - gamepad1.left_trigger));

    }

    public void intakeUp() {
        droptakeRight.setPosition(dropTakeDown);
        droptakeLeft.setPosition(1 - dropTakeDown);
    }
    public void on() {
        intakeMotor.setPower(1.0);
    }

    public void intakeDown() {
        droptakeRight.setPosition(dropTakeDown);
        droptakeLeft.setPosition(1 - dropTakeDown);
    }
}