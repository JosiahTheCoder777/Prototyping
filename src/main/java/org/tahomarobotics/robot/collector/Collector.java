package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Collector extends SubsystemIF {
    private static final Collector INSTANCE = new Collector();
    public static Collector getInstance() {
        return INSTANCE;
    }
    private boolean shouldEject = false;
    private boolean shouldCollect = false;
    private boolean shouldDeploy = false;

    // MOTORS
    private final TalonFX deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
    private final TalonFX deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
    private final TalonFX collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

    // CONTROL REQUESTS
    // Setting up the controls
    private final MotionMagicVoltage deploymentControl = new MotionMagicVoltage(CollectorConstants.STOW_POSITION);
    private final MotionMagicVoltage collectionControl = new MotionMagicVoltage(0);

    private DeploymentState deploymentState = DeploymentState.ZEROING,
            preEjectState;
    private CollectionState collectionState = CollectionState.DISABLED;

    private Collector() {
        deployLeft.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.setInverted(true);
        collectMotor.getConfigurator().apply(CollectorConstants.collectMotorConfiguration);
    }
    // STATUS SIGNALS
    @Override

    public void onTeleopInit(){new CollectorZeroCommand().schedule();}
    // A large conditional that checks whether specific buttons are pressed and does specific actions based on the buttons pressed.
    @Override
    public void periodic() {
        boolean isLeftDPadPressed = false;
        boolean isLeftBumperPressed = false;
        boolean isLeftTriggerPressed = false;

        switch(deploymentState) {
            case -> DEPLOYED -> {
                if (!shouldDeploy) deploymentStow();
                if (isLeftDPadPressed) deploymentEject();
            }
            case EJECT -> {
                if (!isLeftDPadPressed) deploymentEject();
            }
            case STOWED -> {
                if (shouldDeploy) deploymentDeploy();
                if (isLeftDPadPressed) deploymentEject();
            }
            case ZEROING -> {
            }
        }
        switch (collectionState) {
            case COLLECTING -> {
                if (!shouldCollect) {
                    collectionDisabled();
                }
                if (shouldCollect) {
                    collectionEject();
                }
            }
            case EJECTING -> {
            }
            case DISABLED -> {
                if (isLeftTriggerPressed) {collectionCollect();}
                if (isLeftDPadPressed) {collectionEject();}
            }
        }
    }
    // STATE
    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED;
    // Applies the configuration for the motors.
    private Collector Collector() {
        deployLeft.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.getConfigurator().apply(CollectorConstants.deployMotorConfiguration);
        deployRight.setInverted(true);
        collectMotor.getConfigurator().apply(CollectorConstants.collectMotorConfiguration);

    public static Collector getInstance() {
            return INSTANCE;
        }

    // GETTER
    // Booleans to determine whether a state should be called.
    public void setShouldEject(boolean newVal)
        shouldEject = newVal;

    public void setShouldCollect(boolean newVal)
        shouldCollect = newVal;

    public void toggleDeploy(boolean)
        shouldDeploy = !shouldDeploy;

    public double getLeftDeployVelocity()
        return deployLeft.getVelocity().getValueAsDouble();
    }

    public double getRightDeployVelocity() {
        return deployRight.getVelocity().getValueAsDouble();
    }

    // SETTERS
    public void setDeployPosition (double position) {
        deployLeft.setControl().deployControl.withPosition(position);
        deployRight.setControl().deployControl.withPosition(position);
    }

    public void setCollectionVelocity(double velocity) {
        deployMotor.setControl(deployControl.withVelocity(velocity));
    }

    public void setDeployVoltage(double voltage) {
        deployLeft.setVoltage(voltage);
        deployRight.setVoltage(voltage);
    }

    public void zeroDeploys() {
        deployLeft.setPosition(CollectorConstants.ZERO_POSITION);
        deployRight.setPosition(CollectorConstants.ZERO_POSITION);
    }
    // bendy and the
    // STATE MACHINE
    public void collectionCollect() {
        setCollectionVelocity(CollectorConstants.COLLECT_MAX_RPS);
        collectionState = CollectionState.COLLECTING;
    }

    public void collectionDisabled() {
        collectMotor.stopMotor();
        collectionState = collectionState.DISABLED;
    }

    public void collectionEject() {
        setCollectionVelocity(CollectorConstants.COLLECT_MAX_RPS);
        collectionState = collectionState.EJECTING;
    }

    public void deploymentDeploy() {
        setDeployPosition(CollectorConstants.DEPLOY_MAX_RPS);
        preEjectState = deploymentState;
        deploymentState = CollectionState.DEPLOYED;
    }

    public void deploymentEject() {
        setDeployPosition(CollectorConstants.EJECT_POSITION);
        deploymentState = deploymentState.EJECTING;
    }

    public void deploymentUneject() {
        switch(preEjectState) {
            case DEPLOYED -> deploymentDeploy();
            case STOWED -> deploymentState.STOWED;
            default -> {}
        }
    }

    deploymentStow() {
        setDeployPosition(CollectorConstants.STOW_POSITION);
        preEjectState = deploymentState;
        deploymentState = deploymentState.STOWED;
    }
    public void deploymentCollect() {
    }

    // STATES

    public enum CollectionState {
        COLLECTING,
        DISABLED,
        EJECTING,
        ZEROING
    }

    public enum DeploymentState {
        DEPLOYED,
        STOWED,
        EJECT
    }
}




