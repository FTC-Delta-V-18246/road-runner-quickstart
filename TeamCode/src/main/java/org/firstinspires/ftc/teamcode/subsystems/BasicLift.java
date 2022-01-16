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
    public static double MID = -450;
    public static double HIGH = -670;
    public static double SHARED = 400;
    public static double INTAKE = 0;

    public static final double TICKS_PER_REV = 28 * 13.7;
    public static final double GEAR_RATIO = 1;
    public static double kP = -0.0017;
    public static double kF = -0.001;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    private LinearOpMode opMode;
    boolean wasPressedA = false;
    boolean wasPressedB = false;
    private double error = 0;
    private double power = 0;
    public ElapsedTime rruntime;

    public Timer extendTimer;

    public static liftState getState() {
        return state;
    }

    public enum liftState {
        INTAKE,
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
        rruntime = new ElapsedTime();
        liftReset();
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

    public void liftShared() {
        target = SHARED;
    }

    public void liftIntake() {
        target = INTAKE;
    }


    @Override
    public void update(Robot robot) {
        robot.drive.odoRetract();
        robot.deposit.turretNeutral();

        switch (state) {
            case INTAKE:
                liftIntake();
                robot.v4b.intake();
                robot.intake.intakeDown();
                robot.deposit.close();

                if (gamepad1.y) {
                    liftHigh();

                    //extendTimer = new Timer(rruntime, 3.0);

                    state = liftState.DEPOSIT;
                }
                //if(state == liftState.HIGH&&extendTimer.timeUp()){robot.v4b.deposit();}
                //if(state == liftState.MID&&extendTimer.timeUp()){robot.v4b.deposit();}
                //if(state == liftState.SHARED&&extendTimer.timeUp()){robot.v4b.deposit();}

                if (gamepad1.x) {
                    liftMid();
                    robot.v4b.deposit();
                    state = liftState.DEPOSIT;
                }
                /*if (gamepad1.a){
                    robot.v4b.deposit();
                    //extendTimer = new Timer(rruntime, 3.0);
                    liftShared();
                    state = liftState.DEPOSIT;
                }*/
                break;

            case DEPOSIT:
                if (gamepad1.b) {
                    robot.v4b.deposit();
                }
                if (gamepad1.dpad_right) {
                    robot.deposit.open();
                }
                if (gamepad1.a) {
                    robot.deposit.close();
                    robot.v4b.intake();
                    //state = liftState.INTAKE;

                }
                if (gamepad1.dpad_down) {
                    wasPressedA = false;
                    state = liftState.INTAKE;
                }
                break;
        }
        updatePID(target);

    }
    public void updatePID(double target) {
        error = target - encoderTicksToDegrees(lift1.getCurrentPosition());
        power = error * kP + Math.signum(error)*kF;

        if (Math.abs(error) < 75) {
            power = 0;
        }

        power = Range.clip(power, -0.8, 0.8);
        lift1.setPower(-power);
        lift2.setPower(power);
    }

    public static double encoderTicksToDegrees(double ticks) {
        return ((ticks / TICKS_PER_REV) * GEAR_RATIO * 360.0);
    }
    public void ttelemetry(LinearOpMode opMode){
        opMode.telemetry.addData("",power);
        opMode.telemetry.addData("",error);
        opMode.telemetry.addData("", lift2.getCurrentPosition());
        opMode.telemetry.addData("", lift1.getCurrentPosition());
    }
}