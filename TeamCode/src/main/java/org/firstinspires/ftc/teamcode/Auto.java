/* Copyright (c) 2017 FIRST. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided that
* the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list
* of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice, this
* list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* Neither the name of FIRST nor the names of its contributors may be used to endorse or
* promote products derived from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
* This class contains all of the methods we need for autonomous.
*/

public class Auto extends LinearOpMode {
   OmniHardware hardware;
   RelicRecoveryVuMark KEY;
   public final double CRYPTOBOX_COL_WIDTH = 7.5;
   public void runOpMode() {}

   public Auto(OmniHardware hardware) {
      this.hardware = hardware;
   }

   // Encoder methods
   public int neverestEncoderVal(int motorRotations) {
      return hardware.NEVEREST_TICKS_PER_REV * motorRotations;
   }

   /*
    * The following method:
      * Takes in a distance in inches, the wheel circumference in inches, and the ticks per revolution of the motor's encoder
      * Outputs the amount of encoder ticks to be applied in the driveDistance or strafeDistance method in autonomous
    * For example, if we wanted a robot with wheels of circumference 10 in and neverest motors to travel 100 inches with power 0.5, we'd call the method:
      * driveDistance(1, 0.5, encoderTicksForDist(100, 10, hardware.NEVEREST_TICKS_PER_REV));
    * This was derived from the equation(s): distance = circumference x revolutions, revolutions = encoderTicks/encoderTicksPerRev
   */
   public int encoderTicksForDist(double distance, double circumference, int ticksPerRev) {
      return (int) (distance * ticksPerRev / circumference);
   }

   private void resetEncoder(DcMotor motor) {
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   }

   // @param direction: 1 is right, -1 is left
   // @param distance: distance we want robot to travel measured in encoder ticks
   public void driveDistance(int direction, double power, int distance) {
      resetEncoder(hardware.backLeftMotor);
      resetEncoder(hardware.backRightMotor);

      // Set mode to RUN_TO_POSITION
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set target position for encoders to run to
      hardware.backLeftMotor.setTargetPosition(distance);
      hardware.backRightMotor.setTargetPosition(distance);

      // Apply power to motors
      hardware.backLeftMotor.setPower(direction * power);
      hardware.backRightMotor.setPower(direction * power);
      
      while (hardware.backLeftMotor.isBusy() && hardware.backRightMotor.isBusy()) {
          // Wait
      }

      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);
   }

   // @param direction: 1 is right, -1 is left
   // @param distance: distance we want robot to travel measured in encoder ticks
   public void strafeDistance(int direction, double power, int distance) {
      resetEncoder(hardware.middleMotor);

      // Set mode to RUN_TO_POSITION
      hardware.middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set target position for encoders to run to
      hardware.middleMotor.setTargetPosition(-distance);

      // Apply power to motors
      hardware.middleMotor.setPower(direction * power);

      while (hardware.middleMotor.isBusy()) {
          // Wait
      }

      hardware.middleMotor.setPower(0);
   }

   /* Below are backup drive and strafe methods that don't use encoders and instead use a time in milliseconds */
   
   private void drive(int direction, double power, long milliseconds) throws InterruptedException {
      hardware.backLeftMotor.setPower(direction * power);
      hardware.backRightMotor.setPower(direction * power);

      sleep(milliseconds);

      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);
   }
   public void driveForward(double power, long milliseconds) throws InterruptedException {
      drive(1, power, milliseconds);
   }
   public void driveBackward(double power, long milliseconds) throws InterruptedException {
      drive(-1, power, milliseconds);
   }
   
   public void strafeRight(double power, long milliseconds) throws InterruptedException {
      hardware.middleMotor.setPower(power);
      sleep(milliseconds);
      hardware.middleMotor.setPower(0);
   }
   public void strafeLeft(double power, long milliseconds) throws InterruptedException {
      hardware.middleMotor.setPower(-power);
      sleep(milliseconds);
      hardware.middleMotor.setPower(0);
   }

   public void calibrateGyro() {
      hardware.gyroSensor.calibrate(); // Sets Gyro Sensor to 0
      while(hardware.gyroSensor.isCalibrating()) {
          // WAIT - Gyro Sensor is calibrating
      }
   }

   // Corrects the angle of the robot back to original calibration
   public void correctAngle() {
      int startingAngle = hardware.gyroSensor.getHeading(); // angle of the robot at the beginning of the method's execution
      double pow = 1.0; // Power applied to the motors
      int direction = 0; // Direction we will turn in
      if (startingAngle > 180) direction = 1;
      else direction = -1;
      sleep(1000);
      while (hardware.gyroSensor.getHeading() > 5) {
          hardware.backLeftMotor.setPower(pow * direction);
          hardware.backRightMotor.setPower(-pow * direction);

          if (direction == -1) //currentAngle < 180
              pow = (double) hardware.gyroSensor.getHeading()/startingAngle * 0.3 + 0.3;
          else if (direction == 1) //currentAngle > 180
              pow = (double) (360 - hardware.gyroSensor.getHeading())/(360 - startingAngle) * 0.3 + 0.3;
      }
      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);
   }

   // This method turns a certain direction and returns back to starting position - used in knocking the jewel
   private void turnAndBack(int direction, double power) throws InterruptedException {
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      hardware.backLeftMotor.setPower(power * direction);
      hardware.backRightMotor.setPower(-power * direction);
      sleep(2500);
      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);
      hardware.colorServo.setPosition(hardware.SERVO_UP);
      sleep(500);
      correctAngle();
       
      // Set mode back to default
      hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

   // These methods implement the gyro sensor to turn the robot at a certain angle
   // @param angle: positive angle from 0 to 360
   public void turnRight(int angle) throws InterruptedException {
      // May or may not want to calibrate gyro before calling this method
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      int startingAngle = hardware.gyroSensor.getHeading(); // Angle of the robot at beginning of method's execution
      int currAngle = hardware.gyroSensor.getHeading();
      double pow = 1; // Power applied to the motors

      // Turn right
      while (currAngle < angle || (startingAngle > angle && currAngle >= startingAngle)) {
         if (startingAngle > angle) {
             if (currAngle >= startingAngle)
                 pow = (angle + 360 - currAngle)/(angle + 360 - startingAngle) + 0.1;
             else
                 pow = (angle - currAngle)/(angle + 360 - startingAngle) + 0.1;
         }
         else {
             pow = (double) (angle - currAngle)/(angle - startingAngle) + 0.1;
         }

         // Apply power to motors
         hardware.backLeftMotor.setPower(pow);
         hardware.backRightMotor.setPower(-pow);

         // Update currAngle
         currAngle = hardware.gyroSensor.getHeading();
      }

      // Set power back to zero
      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);

      // Set mode back to default
      hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

   // @param direction: 1 for up, -1 for down
   private void movePully(int direction) throws InterruptedException {
      hardware.leftPully.setPower(-0.5 * direction);
      hardware.rightPully.setPower(-0.5 * direction);
      sleep(400);
      hardware.leftPully.setPower(0);
      hardware.rightPully.setPower(0);
   }
   public void raisePully() throws InterruptedException {
      movePully(1);
   }
   public void lowerPully() throws InterruptedException {
      movePully(-1);
   }

   // Metehods pull in or push out glyphs
   public void mecanumsIn(long milliseconds) throws InterruptedException {
      hardware.leftMec.setPower(-1);
      hardware.rightMec.setPower(-1);
      sleep(milliseconds);
      hardware.leftMec.setPower(0);
      hardware.rightMec.setPower(0);
   }
   public void mecanumsOut(long milliseconds) throws InterruptedException {
      hardware.leftMec.setPower(1);
      hardware.rightMec.setPower(1);
      sleep(milliseconds);
      hardware.leftMec.setPower(0);
      hardware.rightMec.setPower(0);
   }

   // Implements the turnAndBack method and the color sensor to knock the jewel
   public void knockJewel(String alliance) throws InterruptedException {
      hardware.colorServo.setPosition(hardware.SERVO_DOWN);
      sleep(2000);
      if (isRed() && alliance.equalsIgnoreCase("blue") || !isRed() && alliance.equalsIgnoreCase("red")) {
          turnAndBack(1, 0.1); // Turn right and back
      }
      else {
          turnAndBack(-1, 0.1); // Turn left and back
      }
   }

   // Method that determines whether color sensor reads red or not
   private boolean isRed() {
      return (hardware.colorSensor.red() > hardware.colorSensor.blue());
   }

   // This method sets a value to KEY based on what is read by the camera - taken from Vuforia
   public void readPictograph(HardwareMap hardwareMap) {
      VuforiaLocalizer vuforia;

      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
      parameters.vuforiaLicenseKey = "AdwaKe7/////AAAAmVQWX/gUQE/gnK+olEmSWA5FCaxNrdY/EyKFLO2afR1IQD4gbnThc6LcCHIJ64hyC2i3n5VRiIRAMGxtKqjI7meHCphQAPrXpH9GomENr/fSXjVUhQao+Zw0/MLQEuTaqNYnp5EI/4oo6LTm/YPgYKOSPaP+tijaydiwNQn4A8zXPfDhkD/q6RTYMzS3UtpOR7WBZJPUBxW9XKim5ekHbYd1Hk2cFTTFAsL0XwycIWhuvHYpVlnZMqWwEnkTqp0o+5TE1FLkAfJ4OOUEfB8sP9kMEcged2/tczAh3GOcjOudp1S9F5xjPFZQX00OLV+QUCPzmT5kkqFBwiS30YR6L8urW2mJG4quq6NnrNYwzn47";
      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Set to front instead of back since our RC needs to use the selfie camera
      vuforia = ClassFactory.createVuforiaLocalizer(parameters);

      VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
      VuforiaTrackable relicTemplate = relicTrackables.get(0);
      relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

      relicTrackables.activate();
      RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

      // Move to read pictograph
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      hardware.backLeftMotor.setPower(-0.3);
      hardware.backRightMotor.setPower(0.3);
      sleep(1000);
      hardware.backLeftMotor.setPower(0);
      hardware.backRightMotor.setPower(0);

      while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
         // Wait until it decodes the pictograph
         vuMark = RelicRecoveryVuMark.from(relicTemplate);
      }

      // Set mode back to default
      hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      KEY = vuMark; // Set KEY to the RelicRecoveryVuMark read by the pictograph
   }

   // This method decides where to insert the glyph into the cryptobox depending on what pictograph is read
   public void cryptoboxInsertion(RelicRecoveryVuMark vuMark, String alliance) {
      double pow = 0.5;
      if (alliance.equalsIgnoreCase("blue")) {
         if (vuMark == RelicRecoveryVuMark.CENTER)
            strafeDistance(1, pow, encoderTicksForDist(CRYPTOBOX_COL_WIDTH, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
         else if (vuMark == RelicRecoveryVuMark.RIGHT)
             strafeDistance(1, pow, encoderTicksForDist(CRYPTOBOX_COL_WIDTH * 2, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
      } else if (alliance.equalsIgnoreCase("red")) {
         if (vuMark == RelicRecoveryVuMark.CENTER)
             strafeDistance(-1, pow, encoderTicksForDist(-CRYPTOBOX_COL_WIDTH, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
         else if (vuMark == RelicRecoveryVuMark.LEFT)
             strafeDistance(-1, pow, encoderTicksForDist(-CRYPTOBOX_COL_WIDTH * 2, hardware.OMNI_CIRCUMFERENCE, hardware.NEVEREST_TICKS_PER_REV));
      }
   }
}
