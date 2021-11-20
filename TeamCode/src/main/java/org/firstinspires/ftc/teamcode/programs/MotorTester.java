package org.firstinspires.ftc.teamcode.programs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.BasicArm;

@TeleOp
@Config
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicArm arm = new BasicArm(gamepad1, gamepad2);
        arm.init(hardwareMap);

        waitForStart();


        while(opModeIsActive()) {
            arm.updateTest();
        }
    }
}
