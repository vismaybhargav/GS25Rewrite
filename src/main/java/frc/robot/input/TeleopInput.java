package frc.robot.input;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;
import com.google.gson.Strictness;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Common class for providing driver inputs during Teleop.
 *
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 *
 * @param <B> the class of the buttons enum
 * @param <A> the class of the axes enum
 *
 */
public final class TeleopInput<B extends Enum<B>, A extends Enum<A>> {

	/* ======================== Constants ======================== */

	/* ======================== Private variables ======================== */

	private enum ControllerType {
		kPS4Controller("ps4", PS4Controller::new) {
			@Override
			void supplyInputs() {
				for (var button : PS4Controller.Button.values()) {
					getButtonMappings().put(button.name(), button.value);
				}
				for (var axis : PS4Controller.Axis.values()) {
					getButtonMappings().put(axis.name(), axis.value);
				}
			}
		},
		kPS5Controller("ps5", PS5Controller::new) {
			@Override
			void supplyInputs() {
				for (var button : PS5Controller.Button.values()) {
					getButtonMappings().put(button.name(), button.value);
				}
				for (var axis : PS5Controller.Axis.values()) {
					getAxesMappings().put(axis.name(), axis.value);
				}
			}
		},
		kJoystick("joystick", Joystick::new) {
			@Override
			void supplyInputs() {
				for (var button : Joystick.ButtonType.values()) {
					getButtonMappings().put(button.name(), button.value);
				}
				for (var axis : Joystick.AxisType.values()) {
					getAxesMappings().put(axis.name(), axis.value);
				}
			}
		},
		kXboxController("xbox", XboxController::new) {
			@Override
			void supplyInputs() {
				for (var button : XboxController.Button.values()) {
					getButtonMappings().put(button.name(), button.value);
				}
				for (var axis : XboxController.Axis.values()) {
					getAxesMappings().put(axis.name(), axis.value);
				}
			}
		};

		private String name;
		private Function<Integer, GenericHID> generator;
		private Map<String, Integer> buttonMappings = new HashMap<>();
		private Map<String, Integer> axesMappings = new HashMap<>();

		@SuppressWarnings("unused")
		String getName() {
			return name;
		}

		@SuppressWarnings("unused")
		Function<Integer, GenericHID> getGenerator() {
			return generator;
		}

		Map<String, Integer> getButtonMappings() {
			return buttonMappings;
		}

		Map<String, Integer> getAxesMappings() {
			return axesMappings;
		}

		ControllerType(String controllerName, Function<Integer, GenericHID> genericHIDGenerator) {
			this.name = controllerName;
			this.generator = genericHIDGenerator;
			supplyInputs();
		}

		abstract void supplyInputs();
	}

	private enum ButtonInputMethods {
		GET,
		PRESSED,
		RELEASED;
	}

	private Class<B> buttons;
	private Class<A> axes;
	private Map<B, Map<ButtonInputMethods, Supplier<Boolean>>> buttonSuppliers;
	private Map<A, Supplier<Double>> axesSuppliers;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 *
	 * @param buttonTypes the type of buttons, which should be an enum of button names used in the mappings file.
	 * @param axesTypes the type of axes, which should be an enum of axes names used in the mappings file.
	 */
	@SuppressWarnings("unchecked")
	public TeleopInput(Class<B> buttonTypes, Class<A> axesTypes) {
		this.buttons = buttonTypes;
		this.axes = axesTypes;
		buttonSuppliers = new HashMap<>();
		axesSuppliers = new HashMap<>();
		Path mappingsFile = Filesystem.getDeployDirectory().toPath().resolve("mappings.jsonc");
		Gson gson = new Gson();
		var type = new TypeToken<Map<String, Object>>() { }.getType();
		Map<String, Object> jsonMappings;
		try {
			var fileReader = new JsonReader(new FileReader(mappingsFile.toFile()));
			fileReader.setStrictness(Strictness.LENIENT);
			jsonMappings = gson.fromJson(fileReader, type);
			var defaults = (List<Map<String, Object>>) jsonMappings.get("defaults");
			defaults.forEach(this::handleEntry);
			List<String> overrides = (List<String>) jsonMappings.get("profiles");
			for (var override : overrides) {
				Path overrideFile = Filesystem.getDeployDirectory().toPath().resolve(override);
				Map<String, Object> overrideJsonMappings = gson.fromJson(new FileReader(overrideFile.toFile()), type);
				handleEntry(overrideJsonMappings);
			}
		} catch (JsonIOException | JsonSyntaxException | FileNotFoundException | ClassCastException e) {
			throw new RuntimeException("Unable to load controller mappings", e);
		}
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/**
	 * Gets the button value for a specific button.
	 * @param key the button identifier
	 * @return the button value
	 */
	public boolean getButtonValue(B key) {
		return buttonSuppliers.get(key).get(ButtonInputMethods.GET).get();
	}

	/**
	 * Gets the button pressed value for a specific button.
	 * @param key the button identifier
	 * @return the button pressed value
	 */
	public boolean getButtonPressed(B key) {
		return buttonSuppliers.get(key).get(ButtonInputMethods.PRESSED).get();
	}

	/**
	 * Gets the button released value for a specific button.
	 * @param key the button identifier
	 * @return the button released value
	 */
	public boolean getButtonReleased(B key) {
		return buttonSuppliers.get(key).get(ButtonInputMethods.RELEASED).get();
	}

	/**
	 * Gets the axis value for a specific axis.
	 * @param key the axis identifier
	 * @return the axis value
	 */
	public double getAxis(A key) {
		return axesSuppliers.get(key).get();
	}

	/* ======================== Private methods ======================== */

	private void handleEntry(Map<String, Object> obj) {
		var controllerTypeStr = (String) obj.get("controller");
		ControllerType tempControllerType = null;
		for (var controllerTypeEntry : ControllerType.values()) {
			if (controllerTypeEntry.name.equals(controllerTypeStr)) {
				tempControllerType = controllerTypeEntry;
				break;
			}
		}
		ControllerType controllerType = tempControllerType;
		var port = ((Double) obj.get("port")).intValue();
		@SuppressWarnings("unchecked")
		var mappingsStrings = (Map<String, String>) obj.get("mappings");
		GenericHID controller;
		controller = controllerType.generator.apply(port);
		for (var mapping : mappingsStrings.entrySet()) {
			boolean unknownMapping = !controllerType.axesMappings.containsKey(mapping.getValue())
									&& !controllerType.buttonMappings.containsKey(mapping.getValue());
			if (unknownMapping) {
				System.out.printf("Unknown mapping: %s\n", mapping.getValue());
			}
			var axis = returnResultOrNull((val) -> Enum.valueOf(axes, val), mapping.getKey());
			if (axis != null) {
				axesSuppliers.put(axis, () -> controller.getRawAxis(controllerType.axesMappings.get(mapping.getValue())));
				System.out.printf("%s of %s on port %d supplied to axis %s\n", mapping.getValue(), controllerType.name, port, axis.name());
				continue;
			}
			var button = returnResultOrNull((val) -> Enum.valueOf(buttons, val), mapping.getKey());
			if (button != null) {
				int buttonKey = controllerType.buttonMappings.get(mapping.getValue());
				buttonSuppliers.put(button, Map.of(
					ButtonInputMethods.GET, () -> controller.getRawButton(buttonKey),
					ButtonInputMethods.PRESSED, () -> controller.getRawButtonPressed(buttonKey),
					ButtonInputMethods.RELEASED, () -> controller.getRawButtonReleased(buttonKey)
				));
				System.out.printf("%s of %s on port %d supplied to button %s\n", mapping.getValue(), controllerType.name, port, button.name());
				continue;
			}
			if (!unknownMapping) {
				System.out.printf("Unknown action: %s\n", mapping.getKey());
			}
			System.out.printf("Unresolved mapping: %s to %s\n", mapping.getKey(), mapping.getValue());
		}
	}

	private <T, R> R returnResultOrNull(Function<T, R> op, T input) {
		try {
			return op.apply(input);
		} catch (IllegalArgumentException iae) {
			return null;
		}
	}

}
