package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake implements Subsystem {
    Gamepad gamepad1;
    Gamepad gamepad2;
    DcMotor intakeMotor;
    Servo droptakeLeft;
    Servo droptakeRight;

    public static double dropTakeDown;
    public static double dropTakeUp;


    public Intake(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;

    }

    @Override
    public void init(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        droptakeLeft = hw.get(Servo.class, "droptakeL");
//        droptakeRight = hw.get(Servo.class, "droptakeR");
    }

    @Override
    public void update(Robot robot) {
        intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }


    public void intakeUp() {
        droptakeRight.setPosition(dropTakeDown);
        droptakeLeft.setPosition(1 - dropTakeDown);
    }

    public void intakeDown() {
        droptakeRight.setPosition(dropTakeUp);
        droptakeLeft.setPosition(1 - dropTakeUp);
    }
}