package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class BasicLift implements Subsystem {
    DcMotor lift1;
    DcMotor lift2;
    public static double target = 0;
    public static double MID = 0;
    public static double HIGH = 0;
    public static double SHARED = 0;
    public static double INTAKE = 0;
    public static double HOLD = 50;
    public static double DEPOSIT = 50;

    public static final double TICKS_PER_REV = 28 * 13.7;
    public static final double GEAR_RATIO = 1;
    public static double kP = 0.012;
    public static double kF = 0.05;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    boolean wasPressedA = false;
    boolean wasPressedB = false;

    boolean lastRight = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean goDown = false;

    public static liftState getState() {
        return state;
    }

    public enum liftState {
        INTAKE,
        TRANSITION,
        MID,
        HIGH,
        SHARED,
        DEPOSIT
    }

    public enum TurretState {Neutral, Blue, Red}

    public enum TrapDoorState {OPEN, CLOSE}

    static BasicLift.liftState state = BasicLift.liftState.INTAKE;

    public BasicLift(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    ElapsedTime timer;

    double last_y_press = 0.0;
    double last_x_press = 0.0;
    double last_a_press = 0.0;
    double last_b_press = 0.0;
    double last_down_press = 0.0;
    public static double PRESS_TIME_MS = 200;

    @Override
    public void init(HardwareMap hw) {
        lift1 = hw.get(DcMotor.class, "lift1");
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        lift2 = hw.get(DcMotor.class, "lift2");
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        // IMPORTANT
        lift2.setDirection(DcMotor.Direction.REVERSE);

        target = INTAKE;

        timer = new ElapsedTime();
        liftReset();
    }

    public void liftReset() {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void liftHigh() {
        target = HIGH;
    }

    public void liftMid() {
        target = MID;
    }

    public void liftShared() {
        target = SHARED;
    }

    public void liftIntake() {
        target = INTAKE;
    }


    @Override
    public void update(Robot robot) {

        double currentloop = System.currentTimeMillis();
        robot.drive.odoRetract();

        switch (state) {
            case INTAKE:
                robot.v4b.intake();
                liftIntake(); //liftstate
                robot.intake.intakeDown();
                robot.deposit.turretNeutral();
                robot.deposit.close();

                if (gamepad1.y && (currentloop - last_y_press) > PRESS_TIME_MS) {
                    last_y_press = currentloop;
                    robot.v4b.deposit();
                    liftHigh();
                    state = liftState.HIGH;
                }
                if (gamepad1.x && (currentloop - last_x_press) > PRESS_TIME_MS) {
                    last_x_press = currentloop;
                    robot.v4b.deposit();
                    liftMid();
                    state = liftState.MID;
                }
                if (gamepad1.a && (currentloop - last_a_press) > PRESS_TIME_MS) {
                    last_a_press = currentloop;
                    robot.v4b.deposit();
                    liftShared();
                    state = liftState.MID;
                }

/*
                if (gamepad1.b) {
                    wasPressedA = true;
                }
                if (!gamepad1.a && wasPressedA) {
                    wasPressedA = false;
                    state = liftState.HOLD;
                }
                if (!gamepad1.b && wasPressedA) {
                    wasPressedB = false;
                    state = liftState.SHARED;
                }
*/
                break;
            case TRANSITION:
                if (gamepad1.y && (currentloop - last_y_press) > PRESS_TIME_MS) {
                    last_y_press = currentloop;
                    robot.deposit.open();
                    state = liftState.DEPOSIT;
                }
                if (gamepad1.dpad_down && goDown == false && (currentloop - last_down_press) > PRESS_TIME_MS) {
                    last_down_press = currentloop;
                    timer.reset();
                    goDown = true;
                }
                if (timer.milliseconds() > 10 && goDown == true) {
                    robot.v4b.hold();
                }
                if (timer.milliseconds() > 710 && goDown == true) {
                    liftIntake();
                    state = liftState.INTAKE;
                    goDown = false;
                }

            case SHARED:
                liftShared();

                if (gamepad1.b) {
                    wasPressedB = true;
                }

                if (!gamepad1.b && wasPressedA) {
                    wasPressedB = false;
                    liftShared();
                    state = liftState.INTAKE;
                }
                break;


            case DEPOSIT:
                if (gamepad1.x && (currentloop - last_x_press) > PRESS_TIME_MS) {
                    last_x_press = currentloop;
                    robot.deposit.close();
                    state = liftState.TRANSITION;
                }
                break;
        }
            updatePID(target);

    }
    public void updatePID(double target) {
        double error = target - encoderTicksToDegrees(lift1.getCurrentPosition());
        double power = error * kP + Math.copySign(kF, error);

        if (Math.abs(error) < 1.5) {
            power = 0;
        }
        power = Range.clip(power, -0.8, 0.8);
        lift1.setPower(power);
        lift2.setPower(-power);
    }

    public static double encoderTicksToDegrees(double ticks) {
        return ((ticks / TICKS_PER_REV) * GEAR_RATIO * 360.0);
    }
}

