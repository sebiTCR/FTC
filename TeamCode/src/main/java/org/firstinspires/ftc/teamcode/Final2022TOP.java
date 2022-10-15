/* Copyright (c) 2017 FIRST. All rights reserved.
 * YES, THIS IS THE LAST VERSION OF THE TELEOP
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="2022 TeleOP: Final-TOPJ", group="FINAL")

public class Final2022TOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Modular elements
    //RAMP
    private DcMotor m_brat;  //Motor Azvarlire
    private DcMotor m_antebrat;
    private DcMotor m_rotrata;
    //movement stuff and whatever
    //NAMING SCHEME
    //M_RF_MVMT
    //   |
    //   V
    //Motor_RightFront_Movement
    
    //FRONT
    private DcMotor m_rf_mvmt;
    private DcMotor m_lf_mvmt;
    
    //BACK
    private DcMotor m_rb_mvmt;
    private DcMotor m_lb_mvmt;

    private Servo srv_handle_claw;
    private CRServo srv_rot_claw;

    private boolean is_claw_closed = false;
    private boolean reversed_controls = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        m_brat  = hardwareMap.dcMotor.get("M_BRAT");
        m_antebrat  = hardwareMap.dcMotor.get("M_ANTEBRAT");
        m_rotrata = hardwareMap.dcMotor.get("M_ROTRATA");
        
        //FRONT
        m_rf_mvmt = hardwareMap.dcMotor.get("M_RF_MVMT");
        m_lf_mvmt = hardwareMap.dcMotor.get("M_LF_MVMT");
        
        //BACK
        m_rb_mvmt = hardwareMap.dcMotor.get("M_RB_MVMT");
        m_lb_mvmt = hardwareMap.dcMotor.get("M_LB_MVMT");

        //movement
        m_rb_mvmt.setDirection(DcMotor.Direction.REVERSE);
        m_rf_mvmt.setDirection(DcMotor.Direction.REVERSE);


        //aux servos
        
        //srv_rot_claw = hardwareMap.servo.get("SRV_ROT_CLAW");
        srv_rot_claw = hardwareMap.crservo.get("SRV_ROT_CLAW");
        srv_handle_claw = hardwareMap.servo.get("SRV_HANDLE_CLAW");


        srv_rot_claw = hardwareMap.crservo.get("SRV_ROT_CLAW");
        srv_handle_claw = hardwareMap.servo.get("SRV_HANDLE_CLAW");

        //Set Motor Modes
        m_brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_antebrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        // rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // double leftPower;
            // double rightPower;
            
            //Motor Azvarlire
           
            m_brat.setPower(!reversed_controls ? -gamepad2.left_stick_y : gamepad2.left_stick_y);
            
            if(gamepad1.x){
                m_rotrata.setPower(1);
            }
            else{
                m_rotrata.setPower(0);
                
            }

            if(gamepad1.a){
                m_rb_mvmt.setPower(1);
            } else {
                m_rb_mvmt.setPower(0);

            }

            if(gamepad1.b){
                m_rf_mvmt.setPower(-1);
                m_lf_mvmt.setPower(-1);
                sleep(100);
                m_rf_mvmt.setPower( 1);
                m_lf_mvmt.setPower( 1);
            } else {
                m_rf_mvmt.setPower(0);
                m_lf_mvmt.setPower(0);
            }
  
            m_antebrat.setPower(!reversed_controls ? -gamepad2.right_stick_y : gamepad2.right_stick_y);
            
            
            //servos
            if(gamepad2.a)
            {    
                is_claw_closed = !is_claw_closed;
                srv_handle_claw.setPosition(is_claw_closed ? 0.0 : 1.0);
                sleep(200);
                
            }
            else if(gamepad2.y){reversed_controls = !reversed_controls;}
            

            srv_rot_claw.setPower(-gamepad2.left_stick_x / 10);
            
            
            //Movement
            
            //Basic Movement
                m_rf_mvmt.setPower(gamepad1.right_stick_y);
                m_rb_mvmt.setPower(gamepad1.right_stick_y);
                
                //bacc
                m_lf_mvmt.setPower(gamepad1.left_stick_y);
                m_lb_mvmt.setPower(gamepad1.left_stick_y);

            
            //STRAIGHT LEFT/RIGHT (OLD
            
            //ROTATION

            if (gamepad1.left_bumper)
            {
                m_rf_mvmt.setPower(-1);
                m_lb_mvmt.setPower(-1);
                
                
                m_lf_mvmt.setPower(1);
                m_rb_mvmt.setPower(1); 
                
            }
            else if (gamepad1.right_bumper)
            {
                m_rf_mvmt.setPower(1);
                m_lb_mvmt.setPower(1);
                
                m_lf_mvmt.setPower(-1);
                m_rb_mvmt.setPower(-1);       
            }
            else
            {
                m_rf_mvmt.setPower(0);
                m_rb_mvmt.setPower(0);
                
                //bacc
                m_lf_mvmt.setPower(0);
                m_lb_mvmt.setPower(0); 
            }
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double drive = -gamepad1.left_stick_y;
            // double turn  =  gamepad1.right_stick_x;
            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            
            
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
