package frc.robot.subsystems;
/** Importing functions */
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
/** Importing autos, signals, and CANdi */
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.pathplanner.lib.auto.NamedCommands;
/** Importing Driver Station, Timer, Shuffleboard, Smart dashboard, Commands, Constants, and Positions */
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SuperSystemConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.AlgaePositions;
import frc.robot.Constants.SuperSystemConstants.CoralPositions;
/** Importing positions, Log level, Swerve */
import frc.robot.Constants.SuperSystemConstants.Position;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

/**
 * Coordinates all major subsystems: elevator, pivot, wrist, intake, climb, and drivetrain.
 */
public class SuperSystem {
    /** Elevator subsystem */
    public Elevator elevator;
    /** Pivot arm subsystem */
    public Pivot pivot;
    /** Wrist mechanism */
    public Wrist wrist;
    /** Intake roller mechanism */
    public IntakeRoller intakeRoller;
    /** Climber mechanism */
    public ClimbV2 climbMotor;
    /** Drivetrain system */
    public SwerveDrivetrain swerveDrivetrain;

    /** Intake sensor signal */
    public StatusSignal<S1StateValue> intakeSensor;
    
    /** Current superstructure position */
    private PositionEquivalents currentPosition = PositionEquivalents.Stow;
    /** Last known superstructure position */
    private PositionEquivalents lastPosition = PositionEquivalents.Stow;
    
    boolean elevatorWithinRange, pivotWithinRange;

    /** Position suppliers */
    private BooleanSupplier pivotAtPosition, elevatorAtPosition, wristAtPosition, pivotAtPositionWide, elevatorAtPositionWide, wristAtPositionWide, intakeDetected;

    /**
     * Defines the order in which mechanisms execute motion
     */
    public enum ExecutionOrder {
        ALL_TOGETHER,
        ELV_PVT_WRT,
        ELV_WRT_PVT,
        PVT_ELV_WRT,
        PVT_WRT_ELV,
        WRT_ELV_PVT,
        WRT_PVT_ELV,
        WRTELV_PVT,
        WRTPVT_ELV,
        PVT_WRTELV
    }

    /**
     * Enum for current mode of play (Algae or Coral)
     */
    public enum PositionMode {
        Coral,
        Algae,
    }

    private PositionMode positionMode = PositionMode.Coral;

    private boolean isStarted = false;
    private boolean wristSet = false, elevatorSet = false, pivotSet = false;
    private double startTime = 0;

    /**
     * Constructs the SuperSystem with all major subsystems
     */
    public SuperSystem(SwerveDrivetrain swerveDrivetrain, Elevator elevator, Pivot pivot, Wrist wrist, IntakeRoller intakeRoller, CANdi candi, ClimbV2 climbMotor) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.intakeRoller = intakeRoller;
        this.intakeSensor = candi.getS1State(true);
        this.climbMotor = climbMotor;
        this.swerveDrivetrain = swerveDrivetrain;

        pivotAtPosition = () -> pivot.atPosition();
        pivotAtPositionWide = () -> pivot.atPositionWide();
        elevatorAtPosition = () -> elevator.atPosition();
        elevatorAtPositionWide = () -> elevator.atPositionWide();
        wristAtPosition = () -> wrist.atPosition();
        wristAtPositionWide = () -> wrist.atPositionWide();
        intakeDetected = () -> (candi.getS1State().getValue().value == 1);

        ShuffleboardTab tab = Shuffleboard.getTab("Supersystem");
        tab.addBoolean("isStarted", () -> isStarted);
        //(after power on, during Disable mode, motors disabled and not applying brake)
        //(after auto/teleop mode, During disable mode, motor disabled and applying brake)
    }

    // subsystems
    /**
     * Reconfigures motor settings for all subsystems
     */
    public void reConfigureMotors() {
        pivot.configureMotorV1();
        elevator.setMotorConfigs();
        wrist.configurePID(wrist.motorConfigs);
        intakeRoller.configureMotor(intakeRoller.motorConfigs);
    }

    /**
     * Sets the neutral mode for subsystems
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        elevator.setNeutralMode(neutralMode);
        pivot.setNeutralMode(neutralMode);
        wrist.setNeutralMode(neutralMode);
        reConfigureMotors();
    }

    /**
     * Zeros all encoder positions for pivot, elevator, and wrist
     */
    public Command zeroEncoders() {
        return Commands.runOnce(()-> {
            pivot.zeroEncoder();
            elevator.zeroEncoder();
            wrist.zeroEncoder();
        });
    }

    /**
     * Stops all moving mechanisms
     */
    public Command stop() {
        return Commands.runOnce(() -> {
            pivot.stopCommand();
            wrist.stopCommand();
            elevator.stopCommand();
        });
    }

    /**
     * Stops the intake roller
     */
    public Command stopRoller() {
        return intakeRoller.stopCommand();
    }

    /**
     * Drives toward coral and intakes until piece detected
     */
    public Command driveToCoralWithIntake(){
        return Commands.race(
                swerveDrivetrain.driveToCoralCommand("limelight-coral", 8),//change later
                intakeUntilSensed()
        );
    }

    /**
     * Intakes based on active game mode
     */
    public Command intake() {
        return Commands.either(
            intakeUntilSensed(), 
            intakeAlgae(), 
            () -> (positionMode == PositionMode.Coral)
        );
    }

    /**
     * Intakes algae
     */
    public Command intakeAlgae() {
        return intakeRoller.intakeAlgae();
    }

    /**
     * Intakes coral
     */
    public Command intakeCoral() {
        return intakeRoller.intakeCoral();
    }

    /**
     * Intakes coral slowly
     */
    public Command intakeCoralSlow() {
        return intakeRoller.intakeCoralSlow();
    }
   
    // public Command repositionCoral() {
    //     return Commands.sequence(
    //         repositionCoralLeft(),
    //         Commands.waitSeconds(0.3),
    //         repositionCoralRight(),
    //         holdPiece()
    //     );
    // }

    // public Command repositionCoralLeft() {
    //     return Commands.sequence(
    //         algaeRoller.setVoltageCommandLeft(0.5),
    //         algaeRoller.setVoltageCommandRight(-1.0)
    //     );
    // }

    // public Command repositionCoralRight() {
    //     return Commands.sequence(
    //         algaeRoller.setVoltageCommandLeft(-1),
    //         algaeRoller.setVoltageCommandRight(0.5)
    //     );
    // }

    /**
     * Intakes until object is sensed or voltage changes
     */
    public Command intakeUntilSensed() {
        return Commands.sequence(    
            intakeCoral(),
            Commands.race(
                Commands.waitUntil(intakeDetected),
                Commands.waitUntil(() -> intakeRoller.desiredVoltageCoral != RollerConstants.kCoralIntakePower)
            ),
            Commands.waitSeconds(0.01),
            stopRoller()
        );
    }

    /**
     * Intakes until sensed or timeout occurs
     */
    public Command intakeUntilSensed(double timeout) {
        return Commands.race(
            intakeCoral().until(intakeDetected),
            Commands.waitSeconds(timeout)
        );
    }

    /**
     * Stops the roller to hold a piece
     */
    public Command holdPiece() {
        return Commands.runOnce(() ->
            stopRoller()
            // intakeRoller.holdAlgae(),
            // () -> (positionMode == PositionMode.Coral)
        );
    }

    /**
     * Outtakes based on game mode
     */
    public Command outtake() {
        return Commands.either(
            outtakeCoral(), 
            outtakeAlgae(), 
            () -> positionMode == PositionMode.Coral
        );
    }

    /**
     * Outtakes algae
     */
    public Command outtakeAlgae(){
        return intakeRoller.outtakeAlgae();
    }

    /**
     * Outtakes coral
     */
    public Command outtakeCoral() {
        // if (currentPosition == PositionEquivalents.L1 && positionMode == PositionMode.Coral) {
        //     return intakeRoller.outtakeL1();   
        // }
        return intakeRoller.outtakeCoral();
    }

    /**
     * Shoots algae using the set voltage
     */
    public Command shootAlgae() {
        return intakeRoller.setVoltageCommand(4.25);
    } 
    // public Command climbPrep() {
    //     return climbMotor.setVoltageCommand(0.5);
    // }

    /**
     * Starts the climb motor
     */
    public Command climbstart() {
        return climbMotor.startClimb();
    }

    /**
     * Starts the climb motor in grip mode
     */
    public Command climbgrip() {
        return climbMotor.startClimbGrip();
    }

    /**
     * Stops the climb motor
     */
    public Command climbstop() {
        return climbMotor.stopClimb();
    }

    // private double hardclampvoltage = 0.0;
    // public Command climbHardRamp() {
    //     return Commands.sequence(
    //         Commands.run(() -> {
    //             hardclampvoltage -= 1 / 50;
    //             hardclampvoltage = Math.max(hardclampvoltage, ClimbConstants.climbHardClampVoltage);
    //         }),
    //         climbMotor.setVoltageCommand(hardclampvoltage)
    //     );
    // }

    // public Command climbHardClamp() {
    //     return climbMotor.setVoltageCommand(ClimbConstants.climbHardClampVoltage);
    // }

    // public Command climbSoftClamp() {
    //     return climbMotor.setVoltageCommand(-0.4);
    // }

    // public Command stopClimb() {
    //     return climbMotor.setVoltageCommand(0.0);
    // }

    /**
     * Moves system into climb up position
     */
    public Command climbCommandUp() {
        return Commands.sequence(
            // climbPrep(), 
            moveTo(PositionEquivalents.ClimbUp) 
        );
    }

    /**
     * Moves system into climb down position
     */
    
    public Command climbCommandDown() {
        return Commands.sequence(
            // climbHardRamp(), 
            moveTo(PositionEquivalents.ClimbDown) 
        );
    }

    public Command updatePositions(PositionEquivalents position) {
        return Commands.runOnce(() -> {
            if(currentPosition != position) lastPosition = currentPosition;
            currentPosition = position;
        });
    }

    public Command moveTo(PositionEquivalents position) {
        return Commands.sequence(
            updatePositions(position),
            Commands.either(goTo(position.coralPos, lastPosition.coralPos), goTo(position.algaePos, lastPosition.algaePos), () -> (positionMode == PositionMode.Coral))
        );
    }

    // movement
    private Command goTo(Position position, Position previousPosition) {
        Command gotoCommand;
        if (position.intermediateWristPosition == position.finalWristPosition)
            gotoCommand = Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 10.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition)
                              
            );
        else
            gotoCommand = Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 10.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
                wrist.setPositionCommand(position.finalWristPosition)
                
            );
        return gotoCommand;
        // return
            // Commands.either(
            //     Commands.sequence(
            //         preExecute(),
            //         execute(PositionEquivalents.intermediateGround.coralPos.executionOrder, 10.0, 
            //         PositionEquivalents.intermediateGround.coralPos.pivotPosition, PositionEquivalents.intermediateGround.coralPos.elevatorPosition, PositionEquivalents.intermediateGround.coralPos.intermediateWristPosition),
            //         wrist.setPositionCommand(PositionEquivalents.intermediateGround.coralPos.finalWristPosition),
            //         preExecute(),
            //         execute(position.executionOrder, 10.0, 
            //         position.pivotPosition, position.elevatorPosition, position.finalWristPosition)
            //     ), 
            //     gotoCommand,
            //     () -> (position == PositionEquivalents.GroundIntake.coralPos ||
            //     previousPosition == PositionEquivalents.GroundIntake.coralPos));
    }

    public Command moveToAuto(PositionEquivalents position) {
        return Commands.sequence(
            updatePositions(position),
            Commands.either(goToAuto(position.coralPos), goToAuto(position.algaePos), () -> (positionMode == PositionMode.Coral))
        );
    }

    public Command goToAuto(Position position) {
        if (position.intermediateWristPosition == position.finalWristPosition)
            return Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 5.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition)             
        );
        return Commands.sequence(
            preExecute(),
            execute(position.executionOrder, 5.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            wrist.setPositionCommand(position.finalWristPosition)
            
        );
    }

    // game elements
    // public Command moveToCage() { //TODO
    //     return moveTo(NamedPositions.Cage);
    // }

    public Command moveToNet() { //TODO
        return moveTo(PositionEquivalents.L4); //Uses net position from position equivalents
    }

    public void togglePositionMode() {
        if (positionMode == PositionMode.Coral) positionMode = PositionMode.Algae;
        else positionMode = PositionMode.Coral;
    }

    public void setPositionMode(PositionMode mode) {
        positionMode = mode;
    }

    public Command togglePositionModeCommand() {
        return Commands.runOnce(() -> togglePositionMode());
    }

    public Command setPositionModeCoral() {
        return Commands.runOnce(() -> setPositionMode(PositionMode.Coral));
    }

    public Command setPositionModeAlgae() {
        return Commands.runOnce(() -> setPositionMode(PositionMode.Algae));
    }
    
    // public Command moveToProcessor() { //TODO
    //     // note: we may not need this one, because the intake action could cover it.
    //     return moveTo(PositionEquivalents.Processor);
    // }    

    public void initialize() {
        pivot.setEnabled(true);
        wrist.setEnabled(true);
        elevator.setEnabled(true);
        intakeRoller.setEnabled(true);
        climbMotor.setEnabled(true);
        
        pivot.setTargetPosition(PositionEquivalents.Stow.coralPos.pivotPosition);
        elevator.setTargetPosition(0.0);
        wrist.setTargetPosition(WristConstants.kWristOffset);
        intakeRoller.setVoltageCommand(0.0);
            // climbMotor.setVoltageCommand(0.0);
        isStarted = false;
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPosition();
        // pivot.setElevatorLength(elevator.getPosition()); // TODO fake
        elevator.setPivotAngle(curPivotAngle);
        wrist.setPivotAngle(curPivotAngle);
    }

    public Command preExecute()
    {
        return Commands.runOnce(()-> {
            pivot.setTargetPosition(pivot.getPosition());
            elevator.setTargetPosition(elevator.getPosition());
            wrist.setTargetPosition(wrist.getPosition());
        }, pivot, elevator, wrist);
    }

    public Command execute(ExecutionOrder exeOrder, double timeout, 
                           double pivotAngle, double elevatorPosition, double wristAngle)
    {
        return Commands.runEnd(() -> {
            if (!isStarted) {
                isStarted = true;
                startTime = Timer.getFPGATimestamp();
                wristSet = false;
                pivotSet = false;
                elevatorSet = false;
            }

            pivotWithinRange = pivot.atPositionWide();
            if(pivotAngle == PositionEquivalents.GroundIntake.coralPos.pivotPosition) 
                pivotWithinRange = pivot.atPosition();

            elevatorWithinRange = elevator.atPosition();
            
            // TODO move this functionality to each subsystem
            updateDependencies(); 
    
            switch (exeOrder) {
                case ALL_TOGETHER:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    elevator.setTargetPosition(elevatorPosition);
                    wristSet = true;
                    wrist.setTargetPosition(wristAngle);
                    elevatorSet = true;
                    break;
    
                case ELV_PVT_WRT:
                    elevator.setTargetPosition(elevatorPosition);
                    elevatorSet = true;
                    if (elevatorAtPositionWide.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        pivotSet = true;
                        if (pivotAtPositionWide.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                            wristSet = true;
                        }
                    }
                    break;
    
                case ELV_WRT_PVT:
                    elevator.setTargetPosition(elevatorPosition);
                    elevatorSet = true;
                    if (elevatorAtPositionWide.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        wristSet = true;
                        if (wristAtPositionWide.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                            pivotSet = true;
                        }
                    }
                    break;
    
                case PVT_WRT_ELV:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    if (pivotAtPositionWide.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        wristSet = true;
                        if (wristAtPositionWide.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
                            elevatorSet = true;
                        }
                    }
                    break;
    
                case PVT_ELV_WRT:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    if (pivotAtPositionWide.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                        if (elevatorAtPositionWide.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                            wristSet = true;
                        }
                    }
                    break;
    
                case WRT_ELV_PVT:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    if (wristAtPositionWide.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                        if (elevatorAtPositionWide.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                            pivotSet = true;
                        }
                    }
                    break;
    
                case WRT_PVT_ELV:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    if (wristAtPositionWide.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        pivotSet = true;
                        if (pivotAtPositionWide.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
                            elevatorSet = true;
                        }
                    }
                    break;

                case WRTELV_PVT:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    elevator.setTargetPosition(elevatorPosition);
                    elevatorSet = true;
                    if (wristAtPositionWide.getAsBoolean() && elevatorAtPositionWide.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        pivotSet = true;
                    }
                    break;
                
                case WRTPVT_ELV:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    if (wristAtPositionWide.getAsBoolean() && pivotAtPositionWide.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                    }
                    break;

                case PVT_WRTELV:
                pivot.setTargetPosition(pivotAngle);
                pivotSet = true;
                if (pivotAtPositionWide.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        wristSet = true;
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                    }
                    break;
            
                default:
                    break;
            }
        },
        () -> {
            isStarted = false;
            wristSet = false;
            pivotSet = false;
            elevatorSet = false;
        }
        ).until(
            () -> (
                (pivot.atPositionWide() && pivotSet
                && elevator.atPositionWide() && elevatorSet
                && wrist.atPositionWide() && wristSet)
                || (Timer.getFPGATimestamp() - startTime >= timeout)
            )
        );
    }

    public void initShuffleboard(LOG_LEVEL priority){
        if (priority == LOG_LEVEL.OFF) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Supersystem");
        switch (priority) {
            case OFF:
                break;
            case ALL:
                tab.addString("Super System Last Position", () -> lastPosition.toString());
            case MEDIUM:
                tab.addString("Super System Current Position", () -> currentPosition.toString());
                tab.addString("Super System Mode", () -> positionMode.toString()); 
                case MINIMAL:
                tab.addBoolean("Intake Detected", intakeDetected);
                break;
        }
    }

}
