package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.atomic.AtomicReference;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Advancing;

@TeleOp
@Mercurial.Attach
public class Ri1WTeleOp extends OpMode {

    double SHOOT_POWER = 0.6;
    GoBildaPinpointDriver odo;
    enum PIVOT_STATE {
        SHOOT,
        LOAD
    }

    @Override
    public void init() {
        BoundGamepad boundGamepad = new BoundGamepad(new SDKGamepad(gamepad1));
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        DcMotor shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        AtomicReference<PIVOT_STATE> state = new AtomicReference<>(PIVOT_STATE.LOAD);


        boundGamepad.x()
                .onTrue(new Lambda("shootOn")
                        .setInit(() -> shootMotor.setPower(SHOOT_POWER)));
        boundGamepad.y()
                .onTrue(new Lambda("shootOff")
                        .setInit(() -> shootMotor.setPower(0)));
        boundGamepad.dpadUp()
                .onTrue(new Lambda("shootIncrease")
                        .setInit(() -> {
                            SHOOT_POWER += 0.025;
                            shootMotor.setPower(SHOOT_POWER);
                        })
                );
        boundGamepad.dpadDown()
                .onTrue(new Lambda("shootDecrease")
                        .setInit(() -> {
                            SHOOT_POWER -= 0.025;
                            shootMotor.setPower(SHOOT_POWER);
                        })
                );

        Advancing pivot = new Advancing(
                new Lambda("Shoot")
                        .setInit(() -> {
                            pivotServo.setPosition(0.2);
                            state.set(PIVOT_STATE.SHOOT);
                        }),
                new Lambda("Load")
                        .setInit(() -> {
                            pivotServo.setPosition(0.6);
                            state.set(PIVOT_STATE.LOAD);
                        })
        );


        boundGamepad.leftTrigger()
                .conditionalBindState()
                .greaterThan(0.1)
                .bind()
                .onTrue(new Lambda("goShoot")
                        .setInit(() -> {
                            /*if (Math.abs(odo.getVelX(DistanceUnit.CM)) <= 0 || state.equals(PIVOT_STATE.LOAD)) {
                                pivot.schedule();
                            }*/
                            pivot.schedule();
                        })
                );

        telemetry.addData("servo", pivotServo.getPortNumber());
        telemetry.update();



        FieldCentricDrive.INSTANCE.setDefaultCommand(FieldCentricDrive.INSTANCE.robotCentricDriveCommand(boundGamepad));
        //P2PTestDrive.INSTANCE.setDefaultCommand(P2PTestDrive.INSTANCE.driveToPointCommand(boundGamepad));
    }
    @Override
    public void loop() {
        telemetry.addData("pose", odo.getPosition());
        telemetry.update();

    }
}