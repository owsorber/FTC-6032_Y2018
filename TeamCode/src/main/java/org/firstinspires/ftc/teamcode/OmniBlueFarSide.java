package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/* OmniBlueFarSide (far side refers to side furthest from relic recovery zone) */

@Autonomous(name="OmniBlueFarSide", group="Linear Opmode")
public class OmniBlueFarSide extends LinearOpMode {
   OmniHardware hardware;
   Auto auto;

   @Override
   public void runOpMode() throws InterruptedException {
      hardware = new OmniHardware(hardwareMap);
      hardware.initHardware();
      auto = new Auto(hardware);

      hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      waitForStart();
      auto.raisePully();

      telemetry.addLine("Calibrating Gyro sensor");
      telemetry.update();
      auto.calibrateGyro();

      telemetry.addLine("Knocking Jewel");
      telemetry.update();
      auto.knockJewel("blue");

      telemetry.addLine("Reading Pictograph");
      telemetry.update();
      auto.readPictograph(hardwareMap);

      telemetry.addLine("Correcting Angle");
      telemetry.update();
      auto.correctAngle();

      telemetry.addLine("Driving Backward");
      telemetry.update();
      auto.driveDistance(-1, 0.7, auto.encoderTicksForDist(-34, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));

      // Turn around
      telemetry.addLine("Turning to Face Cryptobox");
      telemetry.update();
      auto.turnRight(180);

      telemetry.addLine("Strafing Right");
      telemetry.update();
      auto.cryptoboxInsertion(auto.KEY, "blue");
      //auto.strafeDistance(1, 0.5, auto.encoderTicksForDist(dist[1], hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));

      auto.driveForward(0.1, 1000);
      telemetry.addLine("Placing Glyph");
      telemetry.update();
      auto.mecanumsOut(1000);

      telemetry.addLine("Pushing Glyph to Ensure Placement");
      telemetry.update();
      auto.driveBackward(0.2, 1000);
      auto.driveForward(0.5, 1000);
      auto.driveDistance(-1, 0.3, auto.encoderTicksForDist(-10, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
      auto.lowerPully();

   }
}
