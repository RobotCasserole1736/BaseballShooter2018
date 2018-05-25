package edu.wpi.first.wpilibj.templates;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author FIRSTUser
 */
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.parsing.IUtility;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWM;

public class RobotDrive6Motor implements IUtility {
      /**
     * The location of a motor on the robot for the purpose of driving
     */
    public static class MotorType {

        /**
         * The integer value representing this enumeration
         */
        public final int value;
        static final int kFrontLeft_val = 0;
        static final int kFrontRight_val = 1;
        static final int kMidLeft_val = 2;
        static final int kMidRight_val = 3;
        static final int kRearLeft_val = 4;
        static final int kRearRight_val = 5;
        /**
         * motortype: front left
         */
        public static final MotorType kFrontLeft = new MotorType(kFrontLeft_val);
        /**
         * motortype: front right
         */
        public static final MotorType kFrontRight = new MotorType(kFrontRight_val);
        /**
         * motortype: mid left
         */
        public static final MotorType kMidLeft = new MotorType(kMidLeft_val);
        /**
         * motortype: mid right
         */
        public static final MotorType kMidRight = new MotorType(kMidRight_val);
        /**
         * motortype: rear left
         */
        public static final MotorType kRearLeft = new MotorType(kRearLeft_val);
        /**
         * motortype: rear right
         */
        public static final MotorType kRearRight = new MotorType(kRearRight_val);

        private MotorType(int value) {
            this.value = value;
        }
    }
    public static final double kDefaultExpirationTime = 0.1;
    public static final double kDefaultSensitivity = 0.5;
    public static final double kDefaultMaxOutput = 1.0;
    protected static final int kMaxNumberOfMotors = 6;
    protected final int m_invertedMotors[] = new int[6];
    protected double m_sensitivity;
    protected double m_maxOutput;
    protected SpeedController m_frontLeftMotor;
    protected SpeedController m_frontRightMotor;
    protected SpeedController m_midLeftMotor;
    protected SpeedController m_midRightMotor;
    protected SpeedController m_rearLeftMotor;
    protected SpeedController m_rearRightMotor;
    protected boolean m_allocatedSpeedControllers;
    protected static boolean kArcadeRatioCurve_Reported = false;
    protected static boolean kTank_Reported = false;
    protected static boolean kArcadeStandard_Reported = false;

    /**
     * Constructor for RobotDrive with 4 motors specified as SpeedController objects.
     * Speed controller input version of RobotDrive (see previous comments).
     * @param rearLeftMotor The back left SpeedController object used to drive the robot.
     * @param midLeftMotor The mid left SpeedController object used to drive the robot
     * @param frontLeftMotor The front left SpeedController object used to drive the robot
     * @param rearRightMotor The back right SpeedController object used to drive the robot.
     * @param midRightMotor The mid right SpeedController object used to drive the robot
     * @param frontRightMotor The front right SpeedController object used to drive the robot.
     */
    public RobotDrive6Motor(SpeedController frontLeftMotor, SpeedController midLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController midRightMotor, SpeedController rearRightMotor) {
        if (frontLeftMotor == null || midLeftMotor == null || rearLeftMotor == null || frontRightMotor == null || midRightMotor == null || rearRightMotor == null) {
            m_frontLeftMotor = m_midLeftMotor = m_rearLeftMotor = m_frontRightMotor = m_midRightMotor = m_rearRightMotor = null;
            throw new NullPointerException("Null motor provided");
        }
        m_frontLeftMotor = frontLeftMotor;
        m_midLeftMotor = midLeftMotor;
        m_rearLeftMotor = rearLeftMotor;
        m_frontRightMotor = frontRightMotor;
        m_midRightMotor = midRightMotor;
        m_rearRightMotor = rearRightMotor;
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = false;
		drive(0, 0);
    }

    /**
     * Drive the motors at "speed" and "curve".
     *
     * The speed and curve are -1.0 to +1.0 values where 0.0 represents stopped and
     * not turning. The algorithm for adding in the direction attempts to provide a constant
     * turn radius for differing speeds.
     *
     * This function will most likely be used in an autonomous routine.
     *
     * @param outputMagnitude The forward component of the output magnitude to send to the motors.
     * @param curve The rate of turn, constant for different forward speeds.
     */
    public void drive(double outputMagnitude, double curve) {
        double leftOutput, rightOutput;
        
        if(!kArcadeRatioCurve_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeRatioCurve);
            kArcadeRatioCurve_Reported = true;
        }
        if (curve < 0) {
            double value = MathUtils.log(-curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude / ratio;
            rightOutput = outputMagnitude;
        } else if (curve > 0) {
            double value = MathUtils.log(curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude / ratio;
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * drive the robot using two joystick inputs. The Y-axis will be selected from
     * each Joystick object.
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), true);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * drive the robot using two joystick inputs. The Y-axis will be selected from
     * each Joystick object.
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     * @param squaredInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean squaredInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), squaredInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     * @param leftStick The Joystick object to use for the left side of the robot.
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), true);
    }

    /**
     * Provide tank steering using the stored robot configuration.https://github.com/imdunne8/RobotCasserole2014Preseason.git
     * This function lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     * @param leftStick The Joystick object to use for the left side of the robot.https://github.com/imdunne8/RobotCasserole2014Preseason.git
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     * @param squaredInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis, boolean squaredInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), squaredInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you directly provide joystick values from any source.
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     * @param squaredInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void tankDrive(double leftValue, double rightValue, boolean squaredInputs) {
        
        if(!kTank_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_Tank);
            kTank_Reported = true;
        }

        // square the inputs (while preserving the sign) to increase fine control while permitting full power
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);
        if(squaredInputs) {
            if (leftValue >= 0.0) {
                leftValue = (leftValue * leftValue);
            } else {
                leftValue = -(leftValue * leftValue);
            }
            if (rightValue >= 0.0) {
                rightValue = (rightValue * rightValue);
            } else {
                rightValue = -(rightValue * rightValue);
            }
        }
        setLeftRightMotorOutputs(leftValue, rightValue);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     * This function lets you directly provide joystick values from any source.
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     */
    public void tankDrive(double leftValue, double rightValue) {
        tankDrive(leftValue, rightValue, true);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given a single Joystick, the class assumes the Y axis for the move value and the X axis
     * for the rotate value.
     * (Should add more information here regarding the way that arcade drive works.)
     * @param stick The joystick to use for Arcade single-stick driving. The Y-axis will be selected
     * for forwards/backwards and the X-axis will be selected for rotation rate.
     * @param squaredInputs If true, the sensitivity will be decreased for small values
     */
    public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
        // simply call the full-featured arcadeDrive with the appropriate values
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given a single Joystick, the class assumes the Y axis for the move value and the X axis
     * for the rotate value.
     * (Should add more information here regarding the way that arcade drive works.)
     * @param stick The joystick to use for Arcade single-stick driving. The Y-axis will be selected
     * for forwards/backwards and the X-axis will be selected for rotation rate.
     */
    public void arcadeDrive(GenericHID stick) {
        this.arcadeDrive(stick, true);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given two joystick instances and two axis, compute the values to send to either two
     * or four motors.
     * @param moveStick The Joystick object that represents the forward/backward direction
     * @param moveAxis The axis on the moveStick object to use for forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate right/left (typically X_AXIS)
     * @param squaredInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs) {
        double moveValue = moveStick.getRawAxis(moveAxis);
        double rotateValue = rotateStick.getRawAxis(rotateAxis);

        arcadeDrive(moveValue, rotateValue, squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given two joystick instances and two axis, compute the values to send to either two
     * or four motors.
     * @param moveStick The Joystick object that represents the forward/backward direction
     * @param moveAxis The axis on the moveStick object to use for forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate right/left (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
    }

    /**
     * Arcade drive implements single stick driving.
     * This function lets you directly provide joystick values from any source.
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right/left
     * @param squaredInputs If set, decreases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        // local variables to hold the computed PWM values for the motors
        if(!kArcadeStandard_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeStandard);
            kArcadeStandard_Reported = true;
        }

        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Arcade drive implements single stick driving.
     * This function lets you directly provide joystick values from any source.
     * @param moveValue The value to use for fowards/backwards
     * @param rotateValue The value to use for the rotate right/left
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        this.arcadeDrive(moveValue, rotateValue, true);
    }

    /** Set the speed of the right and left motors.
     * This is used once an appropriate drive setup function is called such as
     * twoWheelDrive(). The motors are set to "leftSpeed" and "rightSpeed"
     * and includes flipping the direction of one side for opposing motors.
     * @param leftOutput The speed to send to the left side of the robot.
     * @param rightOutput The speed to send to the right side of the robot.
     */
    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        if (m_rearLeftMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte)0x80;

        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        }
        m_rearLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(-limit(rightOutput) * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        }
        m_rearRightMotor.set(-limit(rightOutput) * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    protected static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        int i;
        for (i=1; i<kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) maxMagnitude = temp;
        }
        if (maxMagnitude > 1.0) {
            for (i=0; i<kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    /**
     * Invert a motor direction.
     * This is used when a motor should run in the opposite direction as the drive
     * code would normally run it. Motors that are direct drive would be inverted, the
     * drive code assumes that the motors are geared with one reversal.
     * @param motor The motor index to invert.
     * @param isInverted True if the motor should be inverted when operated.
     */
    public void setInvertedMotor(MotorType motor, boolean isInverted) {
        m_invertedMotors[motor.value] = isInverted ? -1 : 1;
    }

    /**
     * Set the turning sensitivity.
     *
     * This only impacts the drive() entry-point.
     * @param sensitivity Effectively sets the turning sensitivity (or turn radius for a given value)
     */
    public void setSensitivity(double sensitivity)
    {
            m_sensitivity = sensitivity;
    }

    /**
     * Configure the scaling factor for using RobotDrive with motor controllers in a mode other than PercentVbus.
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput)
    {
            m_maxOutput = maxOutput;
    }


    /**
     * Free the speed controllers if they were allocated locally
     */
    public void free() {
        if (m_allocatedSpeedControllers) {
            if (m_frontLeftMotor != null) {
                ((PWM) m_frontLeftMotor).free();
            }
            if (m_frontRightMotor != null) {
                ((PWM) m_frontRightMotor).free();
            }
            if (m_midLeftMotor != null) {
                ((PWM) m_midLeftMotor).free();
            }
            if (m_midRightMotor != null) {
                ((PWM) m_midRightMotor).free();
            }
            if (m_rearLeftMotor != null) {
                ((PWM) m_rearLeftMotor).free();
            }
            if (m_rearRightMotor != null) {
                ((PWM) m_rearRightMotor).free();
            }
        }
    }
    
    public String getDescription() {
        return "Robot Drive";
    }

    public void stopMotor() {
        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(0.0);
        }
        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(0.0);
        }
        if (m_midLeftMotor != null) {
            m_midLeftMotor.set(0.0);
        }
        if (m_midRightMotor != null) {
            m_midRightMotor.set(0.0);
        }
        if (m_rearLeftMotor != null) {
            m_rearLeftMotor.set(0.0);
        }
        if (m_rearRightMotor != null) {
            m_rearRightMotor.set(0.0);
        }
    }

    protected int getNumMotors()
    {
        int motors = 0;
        if (m_frontLeftMotor != null) motors++;
        if (m_frontRightMotor != null) motors++;
        if (m_midLeftMotor != null) motors++;
        if (m_midRightMotor != null) motors++;
        if (m_rearLeftMotor != null) motors++;
        if (m_rearRightMotor != null) motors++;
        return motors;
    }
}

