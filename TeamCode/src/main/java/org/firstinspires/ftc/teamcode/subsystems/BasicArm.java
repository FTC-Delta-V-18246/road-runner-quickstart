package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class BasicArm implements Subsystem {
    DcMotor arm;
    public static double target = 0;

    public static double MID = 180;
    public static double LOW = 220;
    public static double SHARED = 90;
    public static double INTAKE = 0;
    public static double HOLD = 140;

    public static final double TICKS_PER_REV = 28*19.2;
    public static final double GEAR_RATIO = 2.0/3.0;

    public static double kP = 0.01;
    public static double kF = 0.01;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    boolean wasPressedA = false;

    public enum State {
        INTAKE,
        LOW,
        MID,
        SHARED,
        HOLD
    }

    State state = State.INTAKE;

    public BasicArm(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        arm = hw.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void armReset() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armMid() {
        target = MID;
    }

    public void armLow() {
        target = LOW;
    }

    public void armHold() {
        target = HOLD;
    }

    public void armShared() {
        target = SHARED;
    }

    public void armIntake() {
        target = INTAKE;
    }

    @Override
    public void update(Robot robot) {
        switch(state) {
            case INTAKE:
                armIntake();

                if (gamepad1.a) {
                    wasPressedA = true;
                }

                if (!gamepad1.a && wasPressedA) {
                    wasPressedA = false;
                    state = State.SHARED;
                }
                break;
            case SHARED:
                armShared();

                if (gamepad1.a) {
                    wasPressedA = true;
                }

                if (!gamepad1.a && wasPressedA) {
                    wasPressedA = false;
                    armShared();
                    state = State.SHARED;
                }
                break;
        }

        updatePID(target);
    }

    public void updateTest() {
        switch(state) {
            case INTAKE:
                armIntake();

                if (gamepad1.a) {
                    wasPressedA = true;
                }

                if (!gamepad1.a && wasPressedA) {
                    wasPressedA = false;
                    state = State.SHARED;
                }
                break;
            case SHARED:
                armShared();

                if (gamepad1.a) {
                    wasPressedA = true;
                }

                if (!gamepad1.a && wasPressedA) {
                    wasPressedA = false;
                    armShared();
                    state = State.SHARED;
                }
                break;
        }

        updatePID(target);
    }

    public void updatePID(double target) {
        double error = target - encoderTicksToDegrees(arm.getCurrentPosition());
        double p = kP*error;
        double f = Math.copySign(kF, error);
        double power = p+f;
        power = Range.clip(power, -0.3, 0.3);
        arm.setPower(power);
    }

    public static double encoderTicksToDegrees(double ticks) {
        return ((ticks / TICKS_PER_REV) * GEAR_RATIO * 360.0);
    }
}
