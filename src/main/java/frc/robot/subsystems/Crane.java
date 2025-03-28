package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CraneConstants;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.Segment;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.SparkUtil.PIDFSlot;
import frc.robot.utilities.Time;
import frc.robot.utilities.TunableDouble;
import frc.robot.utilities.TunablePIDF;
import frc.robot.utilities.ValueCache;
import frc.robot.utilities.Vector;

/** The Crane subsystem controls simultaneous movement along pivot and elevator axes.
 *  Mathematically this is modeled as a plane, where the pivot angle in radians is the x axis,
 *  and the elevator height in meters is the y axis. Thus a Transform2d is an (angle,height) point,
 *  aka (a,h). "Positions" are at "points".
 */
public class Crane extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlex m_rightElevatorMotor;

  private final RelativeEncoder m_pivotEncoder;
  private final DutyCycleEncoder m_pivotAbsEncoder;
  private final double m_dutyCycleInitTime;
  private final RelativeEncoder m_elevatorEncoder;
  private final Pololu4079 m_distanceSensor;

  private final SparkClosedLoopController m_pivotPID;
  private final SparkClosedLoopController m_leftElevatorPID;

  private static final TunablePIDF pivotVelocityPIDF = new TunablePIDF("Crane.pivotVelocityPIDF",
    CraneConstants.kPivotMotorVelocityPIDFSlot.pidf());
  private static final TunablePIDF pivotVoltagePIDF = new TunablePIDF("Crane.pivotVoltagePIDF",
    CraneConstants.kPivotMotorVoltagePIDFSlot.pidf());
  private static final TunablePIDF elevatorVelocityPIDF = new TunablePIDF("Crane.elevatorVelocityPIDF",
    CraneConstants.kElevatorMotorVelocityPIDFSlot.pidf());
  private static final TunablePIDF elevatorVoltagePIDF = new TunablePIDF("Crane.elevatorVoltagePIDF",
    CraneConstants.kElevatorMotorVoltagePIDFSlot.pidf());
  private static final TunablePIDF pivotPIDF = new TunablePIDF("Crane.pivotPIDF",
    CraneConstants.kPivotPIDF);
  private static final TunablePIDF elevatorPIDF = new TunablePIDF("Crane.elevatorPIDF",
    CraneConstants.kElevatorPIDF);

  private ProfiledPIDController m_aController = new ProfiledPIDController(
    pivotPIDF.get().p(),
    pivotPIDF.get().i(),
    pivotPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  private ProfiledPIDController m_hController = new ProfiledPIDController(
    elevatorPIDF.get().p(),
    elevatorPIDF.get().i(),
    elevatorPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  
  private static final TunableDouble kS = new TunableDouble("Crane.kS", CraneConstants.kS);
  private static final TunableDouble kG = new TunableDouble("Crane.kG", CraneConstants.kG);
  private static final TunableDouble kV = new TunableDouble("Crane.kV", CraneConstants.kV);
  private ElevatorFeedforward m_elevatorFF = new ElevatorFeedforward(kS.get(), kG.get(), kV.get());

  private double m_stallStartTime = Double.POSITIVE_INFINITY;

  private Translation2d m_goal = new Translation2d(0.0, 0.0);
  private double m_pivotControlFactor; // 1.0 for position-based control.
  private double m_elevatorControlFactor; // 1.0 for position-based control.

  /** Tolerance. Position is in {radians,meters}. Velocity is in {radians,meters}/second. */
  public record Tolerance(double position, double velocity) {}

  private int m_currentSerialNum = 0;
  private boolean m_isVelocityControlled = false;

  private final ValueCache<Double> m_pivotPositionCache;
  private final ValueCache<Double> m_pivotVelocityCache;
  private final ValueCache<Double> m_elevatorPositionCache;
  private final ValueCache<Double> m_elevatorVelocityCache;

  private enum State {
    ESTIMATE_AH,    // Estimate pivot angle and elevator height to inform homing.
    PIVOT_0,        // Rapidly move pivot to 0 (straight out).
    ELEVATOR_RAPID, // Rapidly move elevator down to safe h close to home.
    ELEVATOR_HOME,  // Home the elevator.
    PIVOT_RAPID,    // Rapidly move pivot up to safe a close to home.
    PIVOT_HOME,     // Home the pivot.
    CRANING         // Normal crane operation.
  }
  private State m_state;

  private record DeferredMoveTo(
    Translation2d goal,
    Tolerance pivotTolerance,
    Tolerance elevatorTolerance,
    double pivotVelocityFactor,
    double elevatorVelocityFactor,
    int serialNum
  ) {}
  private Optional<DeferredMoveTo> m_deferredMoveTo = Optional.empty();

  public Crane() {
    m_pivotMotor = new SparkFlex(CraneConstants.kPivotMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_pivotMotor, CraneConstants.kPivotMotorConfig);

    m_leftElevatorMotor = new SparkFlex(CraneConstants.kLeftElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_leftElevatorMotor, CraneConstants.kElevatorMotorConfig);

    m_rightElevatorMotor = new SparkFlex(CraneConstants.kRightElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureFollowerMotor(
      m_rightElevatorMotor,
      CraneConstants.kElevatorMotorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
      m_leftElevatorMotor);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(0.0);
    m_pivotAbsEncoder = new DutyCycleEncoder(CraneConstants.kPivotAbsEncoderChannel,
      2.0 * Math.PI, CraneConstants.kPivotAbsEncoderOffsetRadians);
    m_dutyCycleInitTime = Time.getTimeSeconds();
    m_pivotAbsEncoder.setInverted(CraneConstants.kPivotAbsEncoderInverted);
    m_elevatorEncoder = m_leftElevatorMotor.getEncoder();
    m_elevatorEncoder.setPosition(0.0);
    m_distanceSensor = new Pololu4079(CraneConstants.kDistanceSensorInput);

    m_pivotPID = m_pivotMotor.getClosedLoopController();
    m_leftElevatorPID = m_leftElevatorMotor.getClosedLoopController();

    m_aController.enableContinuousInput(-Math.PI, Math.PI);

    m_pivotPositionCache =
      new ValueCache<Double>(() -> {
        return MathUtil.angleModulus(m_pivotEncoder.getPosition());
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_pivotVelocityCache =
      new ValueCache<Double>(() -> {
        return m_pivotEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorPositionCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getPosition();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorVelocityCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);

    m_state = State.ESTIMATE_AH;
  }

  // Only increase serial number once while controlling with velocity to avoid the serial
  // number rapidly increasing.
  private int allocateSerialNum(double pivotVelocityFactor, double elevatorVelocityFactor) {
    boolean velocityControl = pivotVelocityFactor != 1.0 || elevatorVelocityFactor != 1.0;
    if (velocityControl) {
      if (!m_isVelocityControlled) {
        m_isVelocityControlled = true;
        m_currentSerialNum++;
      }
    } else {
      m_isVelocityControlled = false;
      m_currentSerialNum++;
    }
    return m_currentSerialNum;
  }

  private int moveToNow(Translation2d goal,
      Tolerance pivotTolerance, Tolerance elevatorTolerance,
      double pivotVelocityFactor, double elevatorVelocityFactor,
      int serialNum) {
    m_goal = goal;
    m_aController.setGoal(m_goal.getX());
    m_hController.setGoal(m_goal.getY());
    m_aController.setTolerance(pivotTolerance.position,
      Constants.kDt * pivotTolerance.velocity);
    m_hController.setTolerance(elevatorTolerance.position,
      Constants.kDt * elevatorTolerance.velocity
    );
    m_pivotControlFactor = pivotVelocityFactor;
    m_elevatorControlFactor = elevatorVelocityFactor;
    return serialNum;
  }

  private void moveToNow(Translation2d goal) {
    moveToNow(goal,
      CraneConstants.kDefaultPivotTolerance, CraneConstants.kDefaultElevatorTolerance,
      1.0, 1.0,
      0
    );
  }

  private void movePivotToNow(double pivotAngle) {
    Translation2d goal = new Translation2d(pivotAngle, m_goal.getY());
    moveToNow(goal);
  }

  private void moveElevatorToNow(double elevatorHeight) {
    Translation2d goal = new Translation2d(m_goal.getX(), elevatorHeight);
    moveToNow(goal);
  }

  public int moveTo(Translation2d goal,
      Tolerance pivotTolerance, Tolerance elevatorTolerance,
      double pivotVelocityFactor, double elevatorVelocityFactor) {
    int serialNum = allocateSerialNum(pivotVelocityFactor, elevatorVelocityFactor);
    if (m_state != State.CRANING) {
      m_deferredMoveTo = Optional.of(new DeferredMoveTo(
        goal, pivotTolerance, elevatorTolerance, pivotVelocityFactor, elevatorVelocityFactor,
        serialNum));
      return serialNum;
    }
    return moveToNow(goal, pivotTolerance, elevatorTolerance, pivotVelocityFactor,
      elevatorVelocityFactor, serialNum);
  }

  public int moveTo(Translation2d goal,
      Tolerance pivotTolerance, Tolerance elevatorTolerance) {
    return moveTo(goal, pivotTolerance, elevatorTolerance,
      1.0, 1.0);
  }

  public int moveTo(Translation2d goal) {
    return moveTo(goal,
      CraneConstants.kDefaultPivotTolerance, CraneConstants.kDefaultElevatorTolerance);
  }

  private void updatePivotGoal(double pivotAngle) {
    Translation2d goal = new Translation2d(pivotAngle, m_goal.getY());
    m_goal = goal;
  }

  public int movePivotTo(double pivotAngle) {
    Translation2d goal = new Translation2d(pivotAngle, m_goal.getY());
    return moveTo(goal);
  }

  private void updateElevatorGoal(double elevatorHeight) {
    Translation2d goal = new Translation2d(m_goal.getX(), elevatorHeight);
    m_goal = goal;
  }

  public int moveElevatorTo(double elevatorHeight) {
    Translation2d goal = new Translation2d(m_goal.getX(), elevatorHeight);
    return moveTo(goal);
  }

  private void moveTo(Translation2d goal,
      double pivotVelocityFactor, double elevatorVelocityFactor) {
    moveTo(goal,
      CraneConstants.kDefaultPivotTolerance, CraneConstants.kDefaultElevatorTolerance,
      pivotVelocityFactor, elevatorVelocityFactor);
  }

  public void move(double pivotVelocityFactor, double elevatorVelocityFactor) {
    if (pivotVelocityFactor != 0.0 || elevatorVelocityFactor != 0.0) {
      Translation2d velocity = new Translation2d(
        pivotVelocityFactor * CraneConstants.kPivotMaxSpeedRadiansPerSecond,
        elevatorVelocityFactor * CraneConstants.kElevatorMaxSpeedMetersPerSecond
      );
      Vector v = new Vector(getPosition(), velocity.getAngle());
      for (Segment boundary : CraneConstants.kBoundaries) {
        Optional<Translation2d> goalOpt = boundary.intersection(v);
        if (goalOpt.isPresent()) {
          Translation2d goal = goalOpt.get();
          moveTo(goal, pivotVelocityFactor, elevatorVelocityFactor);
          return;
        }
      }
      System.out.printf("move(%.2f, %.2f) missed all boundaries\n",
        pivotVelocityFactor, elevatorVelocityFactor);
    }

    // Zero velocity, or outside boundaries; move to (i.e. stay at) current position.
    moveTo(getPosition());
  }

  public void movePivot(double pivotVelocityRadiansPerSecond) {
    move(pivotVelocityRadiansPerSecond, 0.0);
  }

  public void moveElevator(double elevatorVelocityMetersPerSecond) {
    move(0.0, elevatorVelocityMetersPerSecond);
  }

  private boolean pivotAtGoal() {
    return m_aController.atGoal();
  }

  private boolean elevatorAtGoal() {
    return m_hController.atGoal();
  }

  public Optional<Integer> atGoal() {
    if (pivotAtGoal() && elevatorAtGoal()) {
      return Optional.of(m_currentSerialNum);
    } else {
      return Optional.empty();
    }
  }

  /* Dynamically scale the a,h controller constraints such that the combined a,h component
   * movements combine to follow a "straight" line, i.e. the component movements complete
   * simultaneously. */
  private void scaleAHConstraints(Translation2d position, Translation2d deviation) {
    // Estimate pivot,elevator movement time, ignoring current velocity, as the basis of constraint
    // factors. Acceleration can be legitimately ignored since it proportionally affects the axes.
    double pivotTime = Math.abs(deviation.getX())
      / (CraneConstants.kPivotMaxSpeedRadiansPerSecond * m_pivotControlFactor);
    double elevatorTime = Math.abs(deviation.getY())
      / CraneConstants.kElevatorMaxSpeedMetersPerSecond * m_elevatorControlFactor;
    double maxTime = Math.max(pivotTime, elevatorTime);
    double aFactor = pivotTime / maxTime;
    double hFactor = elevatorTime / maxTime;
    m_aController.setConstraints(new TrapezoidProfile.Constraints(
      aFactor * CraneConstants.kPivotMaxSpeedRadiansPerSecond * m_pivotControlFactor,
      aFactor * CraneConstants.kPivotMaxAccelerationRadiansPerSecondSquared * m_pivotControlFactor
    ));
    m_hController.setConstraints(new TrapezoidProfile.Constraints(
      hFactor * CraneConstants.kElevatorMaxSpeedMetersPerSecond * m_elevatorControlFactor,
      hFactor * CraneConstants.kElevatorMaxAcccelerationMetersPerSecondSquared * m_elevatorControlFactor
    ));
  }

  private void resetCrane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);
    Translation2d velocity = getVelocity();

    scaleAHConstraints(position, deviation);
    m_aController.reset(
      position.getX(),
      velocity.getX()
    );
    m_hController.reset(
      position.getY(),
      velocity.getY()
    );
  }

  private void crane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);

    scaleAHConstraints(position, deviation);
    double aVelocity = m_aController.calculate(position.getX());
    double hVelocity = m_hController.calculate(position.getY());
    m_pivotPID.setReference(aVelocity, ControlType.kMAXMotionVelocityControl,
      CraneConstants.kPivotMotorVelocityPIDFSlot.slot());
    m_leftElevatorPID.setReference(hVelocity, ControlType.kMAXMotionVelocityControl,
      CraneConstants.kElevatorMotorVelocityPIDFSlot.slot(),
      m_elevatorFF.calculateWithVelocities(m_elevatorEncoder.getVelocity(),
      m_hController.getGoal().velocity));
  }

  private void initPivotPosition(double a) {
    m_pivotEncoder.setPosition(a);
    m_pivotPositionCache.flush();
    updatePivotGoal(a);
    resetCrane();
  }

  private void initElevatorPosition(double h) {
    m_elevatorEncoder.setPosition(h);
    m_elevatorPositionCache.flush();
    updateElevatorGoal(h);
    resetCrane();
  }

  private Translation2d getPosition() {
    return new Translation2d(m_pivotPositionCache.get(), m_elevatorPositionCache.get());
  }

  private Translation2d getVelocity() {
    return new Translation2d(m_pivotVelocityCache.get(), m_elevatorVelocityCache.get());
  }

  private Translation2d getDesiredTranslation() {
    return m_goal;
  }

  private Translation2d getDeviation(Translation2d position) {
    return getDesiredTranslation().minus(position);
  }

  private double getPivotAbsEncoderRadians() {
    return MathUtil.angleModulus(m_pivotAbsEncoder.get());
  }

  private double getElevatorLidarHeight() {
    return CraneConstants.kDistanceSensorBaseMeasurement + m_distanceSensor.getDistance();
  }

  private void toStateCraning() {
    if (m_deferredMoveTo.isPresent()) {
      DeferredMoveTo d = m_deferredMoveTo.get();
      moveToNow(
        d.goal,
        d.pivotTolerance,
        d.elevatorTolerance,
        d.pivotVelocityFactor,
        d.elevatorVelocityFactor,
        d.serialNum
      );
      m_deferredMoveTo = Optional.empty();
    }
    m_state = State.CRANING;
  }

  private void toStatePivot0() {
    double a = m_pivotPositionCache.get();
    if (a < 0.0) {
      movePivotToNow(0.0);
      m_state = State.PIVOT_0;
    } else {
      toStateElevatorRapid();
    }
  }

  private void toStateElevatorRapid() {
    double h = m_elevatorPositionCache.get();
    if (h > CraneConstants.kElevatorHomeRapid) {
      moveElevatorToNow(CraneConstants.kElevatorHomeRapid);
      m_state = State.ELEVATOR_RAPID;
    } else {
      toStateElevatorHome();
    }
  }

  private void toStateElevatorHome() {
    // Use low voltage to move downward slowly.
    m_leftElevatorPID.setReference(CraneConstants.kElevatorHomingVoltage, ControlType.kVoltage,
      CraneConstants.kElevatorMotorVoltagePIDFSlot.slot());
    m_state = State.ELEVATOR_HOME;
  }

  private void toStatePivotRapid() {
    double a = m_pivotPositionCache.get();
    if (a < CraneConstants.kPivotHomeRapid) {
      movePivotToNow(CraneConstants.kPivotHomeRapid);
      m_state = State.PIVOT_RAPID;
    } else {
      toStatePivotHome();
    }
  }

  private void toStatePivotHome() {
    m_pivotPID.setReference(CraneConstants.kPivotHomingVoltage, ControlType.kVoltage,
      CraneConstants.kPivotMotorVoltagePIDFSlot.slot());
    m_state = State.PIVOT_HOME;
  }

  @Override
  public void periodic() {
    updateConstants();
    SmartDashboard.putString("Crane state", m_state.toString());
    switch (m_state) {
      case CRANING: {
        crane();
        break;
      }
      case ESTIMATE_AH: {
        // Give the duty cycle encoder time to accurately estimate PWM frequency.
        double currentTime = Time.getTimeSeconds();
        if (currentTime >= m_dutyCycleInitTime
          + CraneConstants.kDutyCycleInitDelaySeconds) {
            double a = getPivotAbsEncoderRadians();
            initPivotPosition(a);
            double h = getElevatorLidarHeight();
            initElevatorPosition(h);
            toStatePivot0();
        }
        break;
      }
      case PIVOT_0: {
        crane();
        if (pivotAtGoal()) {
          toStateElevatorRapid();
        }
        break;
      }
      case ELEVATOR_RAPID: {
        crane();
        if (elevatorAtGoal()) {
          toStateElevatorHome();
        }
        break;
      }
      case ELEVATOR_HOME: {
        // Check if motor amperage is spiked due to a stall condition creating a short circuit.
        if (m_leftElevatorMotor.getOutputCurrent()
            >= CraneConstants.kElevatorMinStalledHomingAmperage) {
          double currentTime = Time.getTimeSeconds();
          if (m_stallStartTime == Double.POSITIVE_INFINITY) {
            m_stallStartTime = currentTime;
          } else if (currentTime >= m_stallStartTime
              + CraneConstants.kElevatorHomingDebounceSeconds) {
            m_stallStartTime = Double.POSITIVE_INFINITY;
            m_leftElevatorMotor.stopMotor();
            initElevatorPosition(CraneConstants.kElevatorHardMin);
            moveElevatorToNow(CraneConstants.kElevatorHome);
            toStatePivotRapid();
          }
        }
        break;
      }
      case PIVOT_RAPID: {
        crane();
        if (pivotAtGoal()) {
          toStatePivotHome();
        }
        break;
      }
      case PIVOT_HOME: {
        // Check if motor amperage is spiked due to a stall condition creating a short circuit.
        if (m_pivotMotor.getOutputCurrent()
            >= CraneConstants.kPivotMinStalledHomingAmperage) {
          double currentTime = Time.getTimeSeconds();
          if (m_stallStartTime == Double.POSITIVE_INFINITY) {
            m_stallStartTime = currentTime;
          } else if (currentTime >= m_stallStartTime
              + CraneConstants.kPivotHomingDebounceSeconds) {
            m_stallStartTime = Double.POSITIVE_INFINITY;
            double a = getPivotAbsEncoderRadians();
            initPivotPosition(a + CraneConstants.kPivotEndoderFlexRadians);
            m_pivotMotor.stopMotor();
            movePivotToNow(CraneConstants.kPivotHome);
            toStateCraning();
          }
        }
        break;
      }
    }
  }

  private void updateConstants() {
    if (pivotVelocityPIDF.hasChanged()) {
      PIDF pidf = pivotVelocityPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kPivotMotorVelocityPIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(pivotVoltagePIDF.get(),
          CraneConstants.kPivotMotorVoltagePIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kPivotMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_pivotMotor, motorConfig);
    }
    if (pivotVoltagePIDF.hasChanged()) {
      PIDF pidf = pivotVoltagePIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pivotVelocityPIDF.get(),
          CraneConstants.kPivotMotorVelocityPIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kPivotMotorVoltagePIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kPivotMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_pivotMotor, motorConfig);
    }
    if (elevatorVelocityPIDF.hasChanged()) {
      PIDF pidf = elevatorVelocityPIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kElevatorMotorVelocityPIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(elevatorVoltagePIDF.get(),
          CraneConstants.kElevatorMotorVoltagePIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kElevatorMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_leftElevatorMotor, motorConfig);
      SparkUtil.configureFollowerMotor(
        m_rightElevatorMotor,
        motorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
        m_leftElevatorMotor
      );
    }
    if (elevatorVoltagePIDF.hasChanged()) {
      PIDF pidf = elevatorVoltagePIDF.get();
      ArrayList<PIDFSlot> pidfSlots = new ArrayList<>() {{
        add(new SparkUtil.PIDFSlot(pidf, CraneConstants.kElevatorMotorVoltagePIDFSlot.slot()));
        add(new SparkUtil.PIDFSlot(elevatorVelocityPIDF.get(),
          CraneConstants.kElevatorMotorVelocityPIDFSlot.slot()));
      }};
      SparkUtil.Config motorConfig = CraneConstants.kElevatorMotorConfig.withPIDFSlots(pidfSlots);
      SparkUtil.configureMotor(m_leftElevatorMotor, motorConfig);
      SparkUtil.configureFollowerMotor(
        m_rightElevatorMotor,
        motorConfig.withInvert(!CraneConstants.kInvertLeftElevatorMotor),
        m_leftElevatorMotor
      );
    }
    if (kS.hasChanged()) {
      m_elevatorFF.setKs(kS.get());
    }
    if (kG.hasChanged()) {
      m_elevatorFF.setKg(kG.get());
    }
    if (kV.hasChanged()) {
      m_elevatorFF.setKv(kV.get());
    }
    if (pivotPIDF.hasChanged()) {
      PIDF pidf = pivotPIDF.get();
      m_aController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    if (elevatorPIDF.hasChanged()) {
      PIDF pidf = elevatorPIDF.get();
      m_hController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }
}