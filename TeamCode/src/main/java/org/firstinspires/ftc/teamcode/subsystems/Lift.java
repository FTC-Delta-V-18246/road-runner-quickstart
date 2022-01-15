/*package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Lift {
    DcMotor motor1;
    DcMotor motor2;

    // lift positions
    public static double highPos = 20;
    public static double midPos = 10;
    public static double lowPos = 0;

    public static PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0);
    public static double kF = 0;

    PIDFController controller;

    // lift constants
    public static double SPOOL_SIZE_IN = 20.0/25.4;
    public static double MOTOR_RATIO = 5.2;
    public static double TICKS_PER_REV = MOTOR_RATIO * 28.0;
    public static double GEAR_RATIO = 20.0/24.0;


    public static double targetPosition = 0;

    public enum Levels {
        HOME,
        LOW,
        MID,
        HIGH
    }

    Levels levels = Levels.HOME;

    public Lift(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");

        // if you need to make sure to reverse as necessary

//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDFController(coeffs, 0, 0, kF);
    }

    public void update() {
        switch(levels) {
            case HOME:
                targetPosition = 0;
                break;
            case LOW:
                targetPosition = 1;
                break;
            case MID:
                targetPosition = 2;
                break;
            case HIGH:
                targetPosition = 3;
                break;
        }

        updatePID(targetPosition);
    }

    public void updatePID(double target) {
        double leftError = target - encoderTicksToInches(motor1.getCurrentPosition());
        double rightError = target - encoderTicksToInches(motor2.getCurrentPosition());


    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
} */