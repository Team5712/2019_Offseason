/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class Shooter {
    //shoot motor controllers - IDs are accurate as of 8/14/2019 - Alex
    CANSparkMax shootMotor1 = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax shootMotor2 = new CANSparkMax(8, MotorType.kBrushless);

    
    public void setSpeed(double speed) {
        this.shootMotor1.set(speed);
        this.shootMotor2.set(speed);
    }


}
