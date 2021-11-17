package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Claw implements Subsystem{

    Gamepad gamepad1;
    Gamepad gamepad2;
    Servo clawLeft;
    Servo clawRight;


    public static double leftopen;
    public static double leftclose;
    public static double rightopen;
    public static double rightclose;

    public State state = State.OPEN;
    public enum State {OPEN, CLOSE}

    public Claw(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;
    }

    @Override
    public void init(HardwareMap hw) {
        clawLeft = hw.get(Servo.class, "clawLeft");
        clawRight = hw.get(Servo.class, "clawRight");
        open();
    }

    @Override
    public void update(Robot robot) {
        switch(state) {
            case OPEN:
                open();

                break;
            case CLOSE:
                close();
                if(gamepad1.b){
                    open();
                }
                break;
        }
    }

    public void open() {
        clawLeft.setPosition(leftopen);
        clawRight.setPosition(rightopen);
    }

    public void close() {
        clawLeft.setPosition(leftclose);
        clawRight.setPosition(rightclose);
    }
}
