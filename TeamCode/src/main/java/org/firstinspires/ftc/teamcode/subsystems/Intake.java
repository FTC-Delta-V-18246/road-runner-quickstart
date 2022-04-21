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
    public static double autoPower = 0.65;
    public static double carouselPower = 0.3;
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
                    carouselBlue(robot);
                }
                if (gamepad1.dpad_right) {
                    carouselRed(robot);
                }
                break;
        }
    }

    public void autoRightOn(Robot robot) {
        intakeRight.setPower(-intakePower);
        intakeRight();

    }
    public void autoLeftOn(Robot robot) {
        intakeLeft.setPower(intakePower);
        intakeLeft();
    }
    public void carouselBlue(Robot robot) {
        intakeLeft.setPower(-carouselPower);
        intakeRight.setPower(carouselPower);
        intakeDown();
    }
    public void carouselRed(Robot robot) {
        intakeLeft.setPower(carouselPower);
        intakeRight.setPower(-carouselPower);
        intakeDown();
    }

    public void autoRED() {
        intakeRight.setPower(autoPower);
    }
    public void autoBLUE() {
        intakeLeft.setPower(-autoPower);
    }

    public void off() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public void on() {
        intakeLeft.setPower(-intakePower);
        intakeRight.setPower(intakePower);
    }

    public void reverse() {
        intakeLeft.setPower(0.7);
        intakeRight.setPower(-0.7);
    }
}