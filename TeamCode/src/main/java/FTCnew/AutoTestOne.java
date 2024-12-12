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

package FTCnew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name="AutoTestOne", group="Robot")

public class AutoTestOne extends LinearOpMode {


    public void runOpMode() {

        RobotHardwareNew robot = new RobotHardwareNew(this);

        robot.Initialize();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //robot.TimedIntakeLiftControl(1000);

        // drive forward 121.92 cm
        robot.DriveCM(0.3, 77);

        //Lift arm (centemeters may be wrong)
        robot.MoveArm(0.4,20);

        //slide out (centemeters may be wrong)
        robot.MoveSlide(0.6,20);

        //Move arm down (centemeters may be wrong)
        robot.MoveArm(0.4,-10);

        //slide in (centemeters may be wrong)
        robot.MoveSlide(0.6,-10);

        //open claw
        robot.AutoClawOpen();

        // drive backwards 12.00 cm
        robot.DriveCM(0.3, -12.00);

        //drive left 121.92 cm
        robot.StrafeLeftCM(0.3, 160);

        //pickup sample

        robot.TurnRight(0.5,118);

        //drive forward 60.96 cm
        robot.DriveCM(0.3, 35.96);

        //put sample in box


    }}
