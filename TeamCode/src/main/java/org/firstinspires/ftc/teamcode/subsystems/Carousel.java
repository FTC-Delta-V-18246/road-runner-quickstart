package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class Carousel implements Subsystem {

    Gamepad gamepad1;
    Gamepad gamepad2;
    DcMotor carouselMotor;
    boolean isSideRed = false;

    public Carousel(Gamepad g1, Gamepad g2) {
        gamepad1 = g1;
        gamepad2 = g2;

    }

    @Override
    public void init(HardwareMap hw) {
        carouselMotor = hw.get(DcMotor.class, "carousel");
    }

    @Override
    public void update(Robot robot) {
        /*if (isSideRed == true && gamepad1.b) {
            carouselMotor.setPower(1);
        } else {
            carouselMotor.setPower(0);
        }
        if (isSideRed == false && gamepad1.a) {
            carouselMotor.setPower(-1);
        } else {
            carouselMotor.setPower(0);
        }*/
    }
        public void updateCaro(double speed) {
            if(gamepad1.left_bumper) {
                carouselMotor.setPower(-speed);
            } else if(gamepad1.right_bumper) {
                carouselMotor.setPower(speed);
            } else {
                carouselMotor.setPower(0);
            }
        }
    public void side(boolean isRed) {
        if (isRed == true) {
            isSideRed = true;
        } else {
            isSideRed = false;
        }
    }
}
