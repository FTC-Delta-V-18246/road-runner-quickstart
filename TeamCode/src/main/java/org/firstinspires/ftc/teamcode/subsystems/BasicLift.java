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
//import org.firstinspires.ftc.teamcode.programs.RedDuck;

@Config
public class BasicLift implements Subsystem {
    DcMotor lift1;
    DcMotor lift2;
    public static double target = 0;
    public static double HIGH = 585;
    public static double MID = 0;
    public static double INTAKE = -25;
    public static double READY = 100;

    public static final double TICKS_PER_REV = 28 * 13.7;
    public static final double GEAR_RATIO = 1;
    public static double kP = -0.0018;
    public static double kF = -0.001;
    public static double dumpTime = 500;
    public static double kickTime = 500;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private double error = 0;
    private double power = 0;
    public ElapsedTime rruntime;

    public boolean ready = false;

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
        KICK,
        RETRACT
    }

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
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);

        lift2 = hw.get(DcMotor.class, "lift2");
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift2.setDirection(DcMotor.Direction.FORWARD);
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
        if (lift1.getCurrentPosition() < READY) {
            ready = true;
        }
    }

    public void liftMid() {
        target = MID;
    }

    public void liftIntake() {

        target = INTAKE - 100;
    }
    public void liftHome() {
        target = INTAKE;
    }

    ElapsedTime LiftTimer = new ElapsedTime();
    ElapsedTime DumpTimer = new ElapsedTime();
    ElapsedTime kickTimer = new ElapsedTime();
    ElapsedTime IntakeReverseTimer = new ElapsedTime();

    @Override
    public void update(Robot robot) {

        switch (state) {
            case INTAKE:
                robot.v4b.intake(robot);
                /*if (gamepad1.dpad_down) {
                    INTAKE = INTAKE + 5;
                    robot.lift.liftReset();
                    robot.lift.update(robot);
                }
                if (gamepad1.dpad_down) {
                    INTAKE = INTAKE - 5;
                    robot.lift.liftReset();
                    robot.lift.update(robot);
                }*/
                liftHome();
                robot.deposit.receive();
                if (gamepad1.right_trigger > 0) {
                    robot.v4b.intake(robot);
                }
                robot.deposit.depositSensor = robot.deposit.ssensor.getDistance(DistanceUnit.INCH);
                if ((robot.deposit.depositSensor <= robot.deposit.distanceMax)) {
                    gamepad1.rumble(300);
                    gamepad2.rumble(300);
                    robot.deposit.close();
                    state = liftState.HOLD;
                }
                if (gamepad2.right_bumper) {
                    state = liftState.HIGH;
                    robot.deposit.close();
                }
                if (gamepad2.y) {
                    robot.v4b.wartime();
                    state = liftState.DEPOSIT;
                }
                if (gamepad2.a) {
                    state = liftState.SHARED;
                }
                LiftTimer.reset();
                IntakeReverseTimer.reset();
                break;
            case HOLD:
                LiftTimer.reset();
                DumpTimer.reset();
                robot.deposit.close();

                if (gamepad1.right_trigger > 0) {
                    robot.v4b.intake(robot);
                    state = liftState.INTAKE;
                }
                if (gamepad2.right_bumper) {
                    state = liftState.HIGH;
                    robot.deposit.close();
                }
                if (gamepad2.y) {
                    robot.v4b.wartime();
                    state = liftState.DEPOSIT;
                }
                if (gamepad2.a) {
                    state = liftState.SHARED;
                }
                while (IntakeReverseTimer.milliseconds() < 400) {
                    robot.intake.reverse();
                }
                break;
            case HIGH:
                robot.v4b.deposit();
                robot.drive.rotatePower = 2.0;
                robot.deposit.turretNeutral();
                if (DumpTimer.milliseconds() > dumpTime) {
                    state = liftState.DEPOSIT;
                    liftHigh();
                }
                break;
            case SHARED:
                robot.v4b.shared();
                state = liftState.DEPOSIT;
                break;
            case DEPOSIT:
                if (gamepad2.b) {
                    robot.deposit.turretREDSHARED();
                }
                if (gamepad2.x) {
                    robot.deposit.turretBLUESHARED();
                }
                if (gamepad2.left_bumper) {
                    state = liftState.KICK;
                    kickTimer.reset();
                }
                break;
            case KICK:
                robot.deposit.kick();
                DumpTimer.reset();
                if (kickTimer.milliseconds() > kickTime) {
                    state = liftState.RETRACT;
                }
                break;
            case RETRACT:
                robot.deposit.turretNeutral();
                liftIntake();
                robot.drive.rotatePower = 1.0;
                if (lift1.getCurrentPosition() < READY && DumpTimer.milliseconds() > 600) {
                    robot.v4b.intake(robot);
                    state = liftState.INTAKE;
                }
                break;
        }
        updatePID(target);
    }

    public void updatePID(double target) {
        error = target - encoderTicksToDegrees(lift1.getCurrentPosition());
        power = error * kP + Math.signum(error) * kF;
        if (Math.abs(error) < 70) { //75
            power = 0;
        }
        power = Range.clip(power, -0.95, 0.95); //0.8
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
        opMode.telemetry.addData("", getState());
    }
}