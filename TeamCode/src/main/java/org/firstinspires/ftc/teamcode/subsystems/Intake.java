package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake implements Subsystem {
    public static Servo droptakeRight;
    public static Servo droptakeLeft;
    Gamepad gamepad1;
    Gamepad gamepad2;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    public static double dropTakeDown = 0.67;
    public static double dropTakeUp = 0;
    public static double intakePower = 1.0;
    public static double autoPower = 0.805;
    public static double spin = 0.2;
    public static double speed = 0.4;
    public static double spintime = 900;
    private double power;
    private boolean lastButton = false;
    private boolean toggle = false;
    private boolean wasPressedA = false;
    private ElapsedTime duckTimer;

    IntakeState state = IntakeState.INTAKE;
    private boolean toggleA;
    private boolean lastA;

    enum IntakeState {
        INTAKE,
        CARO_LEFT,
        CARO_RIGHT
    }

    public Intake(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    public void intakeLeft() {
        droptakeLeft.setPosition(1 - dropTakeDown);
        droptakeRight.setPosition(dropTakeUp);
    }
    public void intakeRight() {
        droptakeLeft.setPosition(1 - dropTakeUp);
        droptakeRight.setPosition(dropTakeDown);
    }
    public void intakeDown() {
        droptakeLeft.setPosition(1 - dropTakeDown);
        droptakeRight.setPosition(dropTakeDown);
    }
    public void intakeUp() {
        droptakeLeft.setPosition(1 - dropTakeDown);
        droptakeRight.setPosition(dropTakeDown);
    }

    @Override
    public void init(HardwareMap hw) {
        toggleA = false;
        lastA = false;

        intakeLeft = hw.get(DcMotor.class, "intakeLeft");
        intakeRight = hw.get(DcMotor.class, "intakeRight");
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        droptakeLeft = hw.get(Servo.class, "droptakeL");
        droptakeRight = hw.get(Servo.class, "droptakeR");

        duckTimer = new ElapsedTime();

    }

    @Override
    public void update(Robot robot) {

        switch (state) {
            case INTAKE:
                power = gamepad1.right_trigger - gamepad1.left_trigger;

                if(gamepad1.a) {
                    lastA = true;
                }

                if(lastA && !gamepad1.a) {
                    lastA = false;
                    toggleA = !toggleA;
                }

                if(toggleA) {
                    intakeLeft();
                    intakeLeft.setPower(-power);
                } else {
                    intakeRight();
                    intakeRight.setPower(power);
                }

                if (gamepad1.dpad_left) {
                    duckTimer.reset();
                    robot.intake.intakeDown();
                    state = IntakeState.CARO_LEFT;
                }
                if (gamepad1.dpad_right) {
                    duckTimer.reset();
                    robot.intake.intakeDown();
                    state = IntakeState.CARO_RIGHT;
                }
                break;
            case CARO_LEFT:
                if (duckTimer.milliseconds() < spintime) {
                    intakeLeft.setPower(spin);
                    intakeRight.setPower(-spin);
                } else if (duckTimer.milliseconds() > spintime && duckTimer.milliseconds() < 1200) {
                    intakeLeft.setPower(speed);
                    intakeRight.setPower(-speed);
                } else {
                    state = IntakeState.INTAKE;
                }
                break;
            case CARO_RIGHT:
                if (duckTimer.milliseconds() < spintime) {
                    intakeRight.setPower(spin);
                    intakeLeft.setPower(-spin);
                } else if (duckTimer.milliseconds() > spintime && duckTimer.milliseconds() < 1200) {
                    intakeRight.setPower(speed);
                    intakeLeft.setPower(-speed);
                } else {
                    state = IntakeState.INTAKE;
                }
                break;
        }
    }

    public void on(Robot robot) {
        intakeLeft.setPower(-intakePower);
        intakeRight.setPower(intakePower);
    }

    public void autoRED() {
        intakeRight.setPower(-autoPower);
    }
    public void autoBLUE() {
        intakeLeft.setPower(-autoPower);
    }

    public void off() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void reverse() {
        intakeLeft.setPower(0.6);
        intakeRight.setPower(-0.6);
    }
}