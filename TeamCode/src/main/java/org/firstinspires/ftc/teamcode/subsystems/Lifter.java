/*

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Robot;


@Config
public class Lifter {
    DcMotor lift1;
    DcMotor lift2;

    public static double SPOOL_SIZE_IN =.6693;
    public static final double TICKS_PER_REV = 28 * 13.7;
    public static final double GEAR_RATIO = (24/20);
    public static double kP = 0.012;
    public static double kF = 0.05;

    private double targett;

    public enum State {INTAKE, NEUTRAL, REDSHARED, BLUESHARED}
    static State state = State.INTAKE;

    boolean lastRight = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean goDown = false;

    public static double MID = 0;
    public static double HIGH = 0;
    public static double SHARED = 0;
    public static double INTAKE = 0;
    public static double HOLD = 50;

    public static State getState() {
        return state;
    }

    Gamepad g1;
    Gamepad g2;

    ElapsedTime timer;
    ElapsedTime capTimer;
    ElapsedTime clawTimer;

    public enum TurretState {Neutral, Blue, Red}
    public enum TrapDoorState {OPEN, CLOSE}

    TurretState capState = TurretState.Neutral;
    TrapDoorState clawState = TrapDoorState.CLOSE;

    double last_lb_press = 0.0;
    double last_rb_press = 0.0;
    public static double PRESS_TIME_MS = 200;

    public Lifter(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        lift1 = hardwareMap.get(DcMotor.class,"LiftLeft");
        lift2 = hardwareMap.get(DcMotor.class,"LiftRight");

        // IMPORTANT

        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        g1 = gamepad1;
        g2 = gamepad2;

        target = INTAKE;

        timer = new ElapsedTime();


    }

    public void update() {

        double currentloop = System.currentTimeMillis();

        if (lift1.getCurrentPosition() <= 1) {
            ();
        }

        switch(state) {
            case HOME:
                if(g1.right_bumper&& (currentloop - last_rb_press) > PRESS_TIME_MS) {
                    last_rb_press = currentloop;
                    horizontalDeposit();
                    target = highPOS;
                    blockerClose();
                    state = State.TRANSITION;
                }
                if(g1.b) {
                    horizontalDeposit();
                    target = mediumPOS;
                    blockerClose();
                    state = State.TRANSITION;
                }
                if(g1.a) {
                    horizontalDeposit();
                    target = lowPOS;
                    blockerClose();
                    state = State.TRANSITION;
                }
                break;
            case TRANSITION:
                if(g1.left_bumper && (currentloop - last_lb_press) > PRESS_TIME_MS) {
                    last_lb_press = currentloop;
                    hopperOpen();
                    state = State.DEPOSIT;
                }
                if (g1.right_bumper && goDown == false && (currentloop - last_rb_press) > PRESS_TIME_MS) {
                    last_rb_press = currentloop;
                    timer.reset();
                    goDown = true;
                }
                if(timer.milliseconds() > 10 && goDown == true) {
                    horizontalHome();
                }
                if(timer.milliseconds() > 710 && goDown == true) {
                    target = homePOS;
                    state = State.HOME;
                    goDown = false;
                }


                break;
            case DEPOSIT:
                if(g1.left_bumper && (currentloop - last_lb_press) > PRESS_TIME_MS) {
                    last_lb_press = currentloop;
                    hopperClose();
                    state = State.TRANSITION;
                }
                break;
        }

        updatePID(target);
        updateClaw();
        updateArm();
    }



    public void high() {
        target = highPOS;
        horizontalDeposit();
        blockerClose();
    }

    public void deposit() {
        state = State.TRANSITION;
    }

    public void mid() {
        target = mediumPOS;
        horizontalDeposit();
        blockerClose();
    }

    public void low() {
        target = lowPOS;
        horizontalDeposit();
        blockerClose();
    }

    public void home() {
        target = homePOS;
        horizontalDeposit();
        blockerClose();
    }

    public void updatePID(double target) {
        double leftError = target - encoderTicksToInches(leftLift.getCurrentPosition());
        double leftPid = leftError * kP + Math.copySign(kF, leftError);

        if (target == 0 && Math.abs(leftError) <= 0.5) {
            leftLift.setPower(0);
            rightLift.setPower(0);
        } else {
            leftPid = Range.clip(leftPid, -0.3, 1);

            leftLift.setPower(leftPid);
            rightLift.setPower(leftPid);
        }
    }

    public void updateClaw() {
        switch(clawState) {
            case OPEN:
                if(g1.dpad_right) {
                    lastRight = true;
                }

                if(!g1.dpad_right && lastRight) {
                    closeClaw();
                    lastRight = false;
                    clawState = ClawState.CLOSE;
                }
                break;
            case CLOSE:
                if(g1.dpad_right) {
                    lastRight = true;
                }

                if(!g1.dpad_right && lastRight) {
                    openClaw();
                    lastRight = false;
                    clawState = ClawState.OPEN;
                }
                break;
        }
    }

    public void updateArm() {
        switch(capState) {
            case STORE:
                if(g1.dpad_up) {
                    lastUp = true;
                }

                //horizontalHome();


                if(g1.dpad_down) lastDown = true;

                if(!g1.dpad_up && lastUp) {
                    capDeposit();
                    lastUp = false;
                    capState = CapState.DEPOSIT;
                }

                if(!g1.dpad_down && lastDown) {
                    capGrab();
                    lastDown = false;
                    capState = CapState.GRAB;
                }
                break;
            case GRAB:
                if(g1.dpad_up) {
                    lastUp = true;
                }
                // horizontalHome();

                if(g1.dpad_down) lastDown = true;

                if(!g1.dpad_up && lastUp) {
                    capDeposit();
                    lastUp = false;
                    capState = CapState.DEPOSIT;
                }

                if(!g1.dpad_down && lastDown) {
                    capStore();
                    lastDown = false;
                    target = homePOS;
                    capState = CapState.STORE;
                    //
                    if (leftLift.getCurrentPosition() <= 15) {

                        blockerOpen();
                    }
                }
                break;
            case DEPOSIT:
                if(g1.dpad_up) {
                    lastUp = true;
                }

                horizontalHome();

                if(g1.dpad_down) lastDown = true;

                if(!g1.dpad_up && lastUp) {
                    capStore();
                    lastUp = false;
                    target = homePOS;
                    state = State.HOME;
                    capState = CapState.STORE;
                    //
                    if (leftLift.getCurrentPosition() <= 15) {

                        blockerOpen();
                    }
                }

                if(!g1.dpad_down && lastDown) {
                    capGrab();
                    lastDown = false;
                    capState = CapState.GRAB;
                }
                break;
        }
    }


    public void horizontalHome() {
        extend.setPosition(in);
        weirdextend.setPosition(in);
    }

    public void horizontalDeposit() {
        extend.setPosition(out);
        weirdextend.setPosition(out);
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    }
*/