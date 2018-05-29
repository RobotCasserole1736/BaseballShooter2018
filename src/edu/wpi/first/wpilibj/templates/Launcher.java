/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author FIRSTUser
 */
public class Launcher {

    Servo latch;
    CANJaguar motorShooter1;
    CANJaguar motorShooter2;
    //inputs
    int servoChannel = 0;
    int JagDeviceNum1 = 0;
    int JagDeviceNum2 = 1;
    //Two throwing Motors
    int motorShooterId1 = 0;
    int motorShooterId2 = 1;
    
    public Launcher() {
        latch = new Servo(servoChannel);
        try {
            motorShooter1 = new CANJaguar(JagDeviceNum1);
            motorShooter2 = new CANJaguar(JagDeviceNum2);
            motorShooter1.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            motorShooter2.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
        
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
            
        }
        
    }
    
    public void runMotor(boolean cmd) {
        
        if (cmd) {
            try {
                motorShooter1.setX(1);
                motorShooter2.setX(1);
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
            
        } else {
            try {
                motorShooter1.setX(0);
                motorShooter2.setX(0);
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
        }
        
    }

    public void openLatch() {
        
        latch.set(1);
        
    }

    public void stopMotor() {
        
    }

    public void closeLatch() {
        latch.set(0);
    }
    
}
