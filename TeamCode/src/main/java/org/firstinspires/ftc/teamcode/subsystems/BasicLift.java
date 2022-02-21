package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.programs.RedDuck;

@Config
public class BasicLift implements Subsystem {
    DcMotor lift1;
    DcMotor lift2;
    public static double target = 0;
    private static double HIGH = -640;
    private static double SHARED = -200;
    private static double MID = -475;
    private static double INTAKE = 50;
    private static double HOLD = -200;
    public static double READY = -300;

    public static final double TICKS_PER_REV = 28 * 13.7;
    public static final double GEAR_RATIO = 1;
    public static double kP = -0.0014;
    public static double kF = -0.001;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private double error = 0;
    private double power = 0;
    public ElapsedTime rruntime;

    public Timer extendTimer;

    public liftState getState() {
        return state;
    }

    public enum liftState {
        INTAKE,
        HOLD,
        MID,
        HIGH,
        SHARED,
        DEPOSIT,
        DEPOSITSHARED,
        DEPOSITSHAREDENDHEIGHT,
        OPENBOX,
        OPENBOXSHARED,
        RETRACTV4BSHARED,
        RETRACTV4B,
        DOWN
    }

    public enum TurretState {Neutral, Blue, Red}

    public enum TrapDoorState {OPEN, CLOSE}

    BasicLift.liftState state = BasicLift.liftState.INTAKE;

    public BasicLift(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    ElapsedTime timer;

    @Override
    public void init(HardwareMap hw) {
        lift1 = hw.get(DcMotor.class, "lift1");
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        lift2 = hw.get(DcMotor.class, "lift2");
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);

        lift2.setDirection(DcMotor.Direction.REVERSE);
        target = INTAKE;
        timer = new ElapsedTime();
        rruntime = new ElapsedTime();
        liftReset();
        state = liftState.INTAKE;
    }

    public void liftReset() {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftHigh() {
        target = HIGH;
    }
    public void liftMid() {
        target = MID;
    }

    public void liftHold() {
        target = HOLD;
    }

    public void liftShared() {
        target = SHARED;
    }

    public void liftIntake() {
        target = INTAKE;
    }




    public static double LiftTime = .8;
    ElapsedTime LiftTimer = new ElapsedTime();
    public static double DumpTime = .6;
    ElapsedTime DumpTimer = new ElapsedTime();
    ElapsedTime IntakeReverseTimer = new ElapsedTime();


    @Override
    public void update(Robot robot) {

        switch (state) {
            case INTAKE:
                liftIntake();
                robot.v4b.intake();
                robot.deposit.close();
                robot.deposit.depositSensor = robot.deposit.ssensor.getDistance(DistanceUnit.INCH);
                if ((robot.deposit.distanceMax >= robot.deposit.depositSensor && robot.deposit.depositSensor >= robot.deposit.distanceMin)) {
                    state = liftState.HOLD;
                }
                if (gamepad1.right_bumper) {
                    state = liftState.HIGH;
                }
                if (gamepad1.y) {
                    state = liftState.SHARED;
                }
                if (gamepad1.a) {
                    state = liftState.RETRACTV4B;
                }
                LiftTimer.reset();
                IntakeReverseTimer.reset();
                break;
            case HOLD:
                liftHold();
                robot.v4b.intake();
                robot.deposit.close();
                LiftTimer.reset();
                if (gamepad1.right_bumper) {
                    state = liftState.HIGH;
                }
                if (gamepad1.y) {
                    state = liftState.SHARED;
                }
                if (gamepad1.a) {
                    state = liftState.RETRACTV4B;
                }
                while (IntakeReverseTimer.seconds() <= 0.3) {
                    robot.intake.reverse();
                }

                    break;
            case HIGH:
                liftHigh();
                robot.deposit.turretNeutral();
                if (lift1.getCurrentPosition() < READY) {
                    state = liftState.DEPOSIT;
                }
                break;
            case SHARED:
                liftMid();
                DumpTimer.reset();
                if (lift1.getCurrentPosition() < READY) {
                    state = liftState.DEPOSITSHAREDENDHEIGHT;
                }
                break;
            case DEPOSITSHAREDENDHEIGHT:
                robot.v4b.deposit();
                if (DumpTimer.seconds() >= DumpTime) {
                    robot.deposit.turretBLUESHARED();
                    liftShared();
                if (gamepad1.left_bumper) {
                    state = liftState.OPENBOXSHARED;
                    LiftTimer.reset();
                }
                if (gamepad1.right_bumper) {
                    state = liftState.HIGH;
                    }
                }
                break;
            case DEPOSIT:
                robot.v4b.deposit();
                if (gamepad1.left_bumper) {
                    state = liftState.OPENBOX;
                }
                if (gamepad1.y) {
                    state = liftState.DEPOSITSHAREDENDHEIGHT;
                }
                break;
            case OPENBOX:
                robot.deposit.open();
                robot.deposit.turretNeutral();
                DumpTimer.reset();
                if (gamepad1.a) {
                    state = liftState.RETRACTV4B;
                }
                break;
            case OPENBOXSHARED:
                robot.deposit.turretBLUESHARED();
                robot.deposit.open();
                DumpTimer.reset();
                if (LiftTimer.seconds() >= 1) {
                    state = liftState.RETRACTV4BSHARED;
                    liftMid();
                }
                break;
            case RETRACTV4BSHARED:
                if (DumpTimer.seconds() >= DumpTime) {
                    robot.deposit.turretNeutral();
                    state = liftState.RETRACTV4B;
                    DumpTimer.reset();
                }
                break;
            case RETRACTV4B:
                robot.v4b.intake();
                robot.deposit.turretNeutral();
                robot.deposit.close();
                if (DumpTimer.seconds() >= DumpTime) {
                    state = liftState.INTAKE;
                }
                break;
        }
        updatePID(target);
    }

    public void updatePID(double target) {
        error = target - encoderTicksToDegrees(lift1.getCurrentPosition());
        power = error * kP + Math.signum(error) * kF;
        if (Math.abs(error) < 75) {
            power = 0;
        }

        power = Range.clip(power, -0.75, 0.75); //0.8
        lift1.setPower(-power);
        lift2.setPower(power);
    }

    public static double encoderTicksToDegrees(double ticks) {
        return ((ticks / TICKS_PER_REV) * GEAR_RATIO * 360.0);
    }

    public void ttelemetry(LinearOpMode opMode) {
        opMode.telemetry.addData("", power);
        opMode.telemetry.addData("", error);
        opMode.telemetry.addData("", lift2.getCurrentPosition());
        opMode.telemetry.addData("", lift1.getCurrentPosition());
    }
}