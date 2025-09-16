package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Advancing;

@TeleOp
@Mercurial.Attach
public class Ri1WTeleOp extends OpMode {

    double SHOOT_POWER = 0.6;

    @Override
    public void init() {
        BoundGamepad boundGamepad = new BoundGamepad(new SDKGamepad(gamepad1));
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        DcMotor shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
                        }),
                new Lambda("Load")
                        .setInit(() -> {
                            pivotServo.setPosition(0.6);
                        })
        );


        boundGamepad.leftTrigger()
                .conditionalBindState()
                .greaterThan(0.1)
                .bind()
                .onTrue(new Lambda("goShoot")
                        .setInit(() -> {
                                pivot.schedule();
                        })
                );

        FieldCentricDrive.INSTANCE.setDefaultCommand(FieldCentricDrive.INSTANCE.robotCentricDriveCommand(boundGamepad));
    }
    @Override
    public void loop() {}
}