package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.BasicArm;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.V4b;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    List<Subsystem> subsystems;

    public Drive drive;
    public Intake intake;
    public Carousel carousel;
    public Claw claw;
    public V4b v4b;
    public BasicArm arm;

    public Robot(HardwareMap hw, Gamepad g1, Gamepad g2) {
        subsystems = new ArrayList<>();
        drive = new Drive(g1, g2);
        intake = new Intake(g1, g2);
        carousel = new Carousel(g1, g2);
//        claw = new Claw(g1, g2);
//        v4b = new V4b(g1, g2);
        arm = new BasicArm(g1, g2);

        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(carousel);
        subsystems.add(arm);
//        subsystems.add(claw);
//        subsystems.add(v4b);


        for (Subsystem s : subsystems) {
            s.init(hw);
        }
    }

    public void update() {
        for (Subsystem s : subsystems) {
            s.update(this);
        }
    }
}