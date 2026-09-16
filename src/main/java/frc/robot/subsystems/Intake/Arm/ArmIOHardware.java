
package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class ArmIOHardware implements ArmIO {

        private static final Angle start_position = Degrees.of(41);

        private static final Angle minangle = Degrees.of(180);
        private static final Angle maxangle = Degrees.of(90);

        private final TalonFX armMotor;
        private final TalonFX followMotor;

        private final StatusSignal<Angle> armPosition;

        private final MotionMagicVoltage armOutput = new MotionMagicVoltage(Degrees.of(0));

        public ArmIOHardware() {

                armMotor = new TalonFX(31, "9427");
                followMotor = new TalonFX(32, "9427");

                armPosition = armMotor.getPosition();

                configure();

                resetEncoder();
        }

        @Override
        public void setPosition(Angle angle) {
                // 使用 Motion Magic 進行平滑軌跡閉環控制，避免手臂因急遽加速而損壞機構
                armMotor.setControl(
                                armOutput.withPosition(angle));
        }

        @Override
        public Angle getPosition() {
                armPosition.refresh();
                return armPosition.getValue();
        }

        @Override
        public void resetEncoder() {
                // 將當前編碼器位置校正為起始角度（上電或歸零時調用）
                armMotor.getConfigurator().setPosition(start_position);
                followMotor.getConfigurator().setPosition(start_position);
        }

        @Override
        public void configure() {
                TalonFXConfiguration armConfig = new TalonFXConfiguration();

                /*
                 * 電流限制配置：
                 * 1. Stator Current Limit (定子電流/馬達端):
                 *    限制馬達線圈內部電流，防止馬達在低速高負載或堵轉時過熱燒毀 (設定 40A)。
                 * 2. Supply Current Limit (電源端/電池輸入電流):
                 *    限制從 PDP/PDH 抽取的總電流，避免急遽加速時瞬間大電流導致電池電壓驟降 (Brownout) (設定 20A)。
                 */
                armConfig.CurrentLimits
                                .withStatorCurrentLimit(40)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true);

                armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                /*
                 * 機構齒輪比轉換 (SensorToMechanismRatio):
                 * 設定馬達轉軸與實際旋轉機構（手臂軸）的齒比 (10:1)。
                 * 設定後，所有位置 (Position) 與速度 (Velocity) 單位都會自動從「馬達轉數」轉換為「機構端轉數 (Mechanism Rotations)」。
                 */
                armConfig.Feedback.SensorToMechanismRatio = 10.0;

                /*
                 * PID 閉環控制參數 (Slot 0):
                 * - kP (比例增益): 依據目標誤差大小產生輸出修正。
                 * - kI (積分增益): 消除長時間存在的穩態微小誤差。
                 * - kD (微分增益): 抑制高速接近目標時的超調與震盪。
                 */
                armConfig.Slot0.kP = 26.1;
                armConfig.Slot0.kI = 226.6;
                armConfig.Slot0.kD = 1.32;

                /*
                 * 重力前饋補償模型 (Gravity Compensation - Arm_Cosine):
                 * 手臂旋轉時受到的重力力矩與夾角餘弦成正比 (力矩 = m * g * L * cos(θ))。
                 * 在 0 度 (水平伸出) 時 cos(0) = 1 重力補償電壓最大；在 90 度 (垂直朝上) 時 cos(90) = 0 無需補償。
                 */
                armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                armConfig.Slot0.kG = 0.0;

                /*
                 * Motion Magic 運動輪廓生成參數 (Trapezoidal / S-Curve Motion Profile):
                 * - MotionMagicCruiseVelocity: 最高巡航速度 (單位: 機構轉數/秒, rotations/s)
                 * - MotionMagicAcceleration: 加速度 (單位: 機構轉數/秒平方, rotations/s^2)
                 * - MotionMagicJerk: 加加速度 (Jerk, 單位: rotations/s^3)，用於平滑加速度曲線 (S-Curve)，減少結構衝擊與晃動。
                 */
                armConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
                armConfig.MotionMagic.MotionMagicAcceleration = 2.0;
                armConfig.MotionMagic.MotionMagicJerk = 10.0;

                /*
                 * 軟體極限保護 (Software Limit Switch):
                 * 在馬達韌體層級限制轉動範圍，當超過門檻時自動禁止該方向輸出，防止手臂撞擊機械硬限位。
                 * 門檻需轉換為「機構轉數」(角度 / 360.0)。
                 */
                armConfig.SoftwareLimitSwitch
                                .withReverseSoftLimitThreshold(
                                                minangle.in(Degrees) / 360.0)
                                .withReverseSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(
                                                maxangle.in(Degrees) / 360.0)
                                .withForwardSoftLimitEnable(true);

                armMotor.getConfigurator().apply(armConfig);
                followMotor.getConfigurator().apply(armConfig);

                /*
                 * 從動馬達 (Follower) 配置:
                 * 設定 followMotor 自動鏡像跟隨 armMotor 的控制指令。
                 * MotorAlignmentValue.Opposed 代表從動馬達與主動馬達在機構上為面對面安裝，轉向需自動相反以同向驅動手臂。
                 */
                followMotor.setControl(
                                new Follower(
                                                armMotor.getDeviceID(),
                                                MotorAlignmentValue.Opposed));
        }
}
