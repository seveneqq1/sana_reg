package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Flower extends SubsystemBase {
    private Servo flower;
    private boolean beka = false;
    public static double closeee = 0.26;
    public static double OPEN_POS = 1, CLOSE_POS = closeee;
    public void init(HardwareMap hw){
        flower = hw.get(Servo.class, "flower");
        closeFlower();
    }
    public void setPos(double pos){
        flower.setPosition(pos);
    }
    public void openFlower(){
        beka = true;
        setPos(OPEN_POS);
    }
    public void closeFlower(){
        beka = false;
        setPos(CLOSE_POS);
    }
    public void toggleFlower(){
        if (isFlowerOpen()) closeFlower();
        else openFlower();
    }
    public boolean isFlowerOpen(){ return beka; }
}
