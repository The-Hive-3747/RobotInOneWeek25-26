package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Color;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.PrismAnimations;

public class TurretLights {
    GoBildaPrismDriver prism;
    PrismAnimations.Solid ball1 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball2 = new PrismAnimations.Solid(Color.ORANGE);
    PrismAnimations.Solid ball3 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid ball4 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid ball5 = new PrismAnimations.Solid(Color.TEAL);
    PrismAnimations.Solid ball6 = new PrismAnimations.Solid(Color.CYAN);

    HardwareMap hardwareMap;
    Telemetry telemetry;
    int brightness = 15;
    int setStartIndex = 0;
    int setStopIndex = 12;



    public TurretLights(HardwareMap hm, Telemetry tm){
        hardwareMap = hm;
        telemetry = tm;
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        prism.setStripLength(12);


    }

    public void redAlliance(){
        ball1.setBrightness(30);
        ball1.setStartIndex(0);
        ball1.setStopIndex(1);
        ball2.setBrightness(30);
        ball2.setStartIndex(2);
        ball2.setStopIndex(3);
        ball3.setBrightness(30);
        ball3.setStartIndex(4);
        ball3.setStopIndex(5);
        ball4.setBrightness(30);
        ball4.setStartIndex(6);
        ball4.setStopIndex(7);
        ball5.setBrightness(30);
        ball5.setStartIndex(8);
        ball5.setStopIndex(9);
        ball6.setBrightness(30);
        ball6.setStartIndex(10);
        ball6.setStopIndex(11);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, ball1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, ball2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, ball3);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, ball4);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, ball5);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, ball6);
    }

}
