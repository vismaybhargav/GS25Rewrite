package frc.robot.systems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIOTalonFX implements ElevatorIO {
	private DCMotor gearbox = DCMotor.getKrakenX60(1);
	private TalonFXSimState talonSimState;

	private LinearSystem<N2, N1, N2> linearSim = LinearSystemId.createElevatorSystem(
			gearbox,
			Units.lbsToKilograms(20),
			Units.inchesToMeters(1),
			15
	);

	private ElevatorSim elevatorSim = new ElevatorSim(
		linearSim,
		gearbox,
		0,
		Units.inchesToMeters(37.3),
		true,
		0
	);

	public ElevatorIOSim(int id, String canBus) {
		super(id, canBus);
		talonSimState = talon.getSimState();
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		inputs = new ElevatorIOData(
			true, 
			t, 0, 0, 0);
	}
}
