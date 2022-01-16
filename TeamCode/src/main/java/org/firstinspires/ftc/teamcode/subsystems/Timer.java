package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

    public class Timer {
        ElapsedTime waitClock;
        double startTime;
        double length;
        public Timer (ElapsedTime waitClock, double length){
            this.waitClock = waitClock;
            this.length = length;
            startTime = this.waitClock.seconds();
        }
        public boolean timeUp(){
            if(waitClock.seconds()-startTime>=length){
                return true;
            }
            else{
                return false;
            }
        }
    }