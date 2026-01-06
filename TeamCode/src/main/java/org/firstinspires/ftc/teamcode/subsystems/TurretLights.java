package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Color;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.PrismAnimations;

public class TurretLights {
    GoBildaPrismDriver prism;
    PrismAnimations.Solid ball1 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball2 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball3 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball4 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball5 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid ball6 = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid blue1 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid blue2 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid blue3 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid blue4 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid blue5 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid blue6 = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.DroidScan shootNow = new PrismAnimations.DroidScan(Color.GREEN);
    PrismAnimations.Solid noShoot = new PrismAnimations.Solid(Color.MAGENTA);

    HardwareMap hardwareMap;
    Telemetry telemetry;
    int brightness = 30;
    int startIndex = 0;
    int stopIndex = 11;



    public TurretLights(HardwareMap hm, Telemetry tm){
        hardwareMap = hm;
        telemetry = tm;
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        prism.setStripLength(12);


    }

    public void readyToShoot(){
        shootNow.setStartIndex(startIndex);
        shootNow.setStopIndex(stopIndex);
        shootNow.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, shootNow);
    }

    public void notReadyToShoot(){
        noShoot.setStartIndex(startIndex);
        noShoot.setStartIndex(stopIndex);
        noShoot.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, noShoot);
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

    public void blueAlliance(){
        blue1.setBrightness(30);
        blue1.setStartIndex(0);
        blue1.setStopIndex(1);
        blue2.setBrightness(30);
        blue2.setStartIndex(2);
        blue2.setStopIndex(3);
        blue3.setBrightness(30);
        blue3.setStartIndex(4);
        blue3.setStopIndex(5);
        blue4.setBrightness(30);
        blue4.setStartIndex(6);
        blue4.setStopIndex(7);
        blue5.setBrightness(30);
        blue5.setStartIndex(8);
        blue5.setStopIndex(9);
        blue6.setBrightness(30);
        blue6.setStartIndex(10);
        blue6.setStopIndex(11);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blue1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, blue2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, blue3);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, blue4);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, blue5);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, blue6);
    }

}
