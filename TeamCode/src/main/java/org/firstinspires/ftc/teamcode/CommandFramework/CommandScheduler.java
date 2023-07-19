package org.firstinspires.ftc.teamcode.CommandFramework;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;

public class CommandScheduler {
	protected HardwareMap hwMap;
	protected ArrayList<Subsystem> subsystems = new ArrayList<>();
	protected ArrayList<Command> activeCommands = new ArrayList<>();
	protected ArrayList<Command> interruptedCommands = new ArrayList<>();
	protected ArrayList<Intent> intents = new ArrayList<>();

	public CommandScheduler(HardwareMap hardwareMap, Subsystem... initSubsystems) {
		hwMap = hardwareMap;
		Collections.addAll(subsystems, initSubsystems);
	}

	public void initAuto() {
		for (Subsystem subsystem : subsystems)
			subsystem.initAuto(hwMap);
	}

	public void initTeleop() {
		for (Subsystem subsystem : subsystems)
			subsystem.initTeleop(hwMap);
	}

	public void shutdown() {
		for (Subsystem subsystem : subsystems)
			subsystem.shutdown();
	}

	public void addIntent(Intent intent) {
	    intents.add(intent);
	}

	public void run() {
		for (Subsystem subsystem : subsystems) {
			subsystem.startTimer();
			subsystem.periodic();
			subsystem.stopTimer();
		}

		// detect intents
		for (Intent intent : intents) {
			if (!intent.isReady()) {
				// skip this intent
				continue;
			}
			if (intent.getCommand().canInterruptOthers) {
				// interrupt all active commands
				for (Command command : activeCommands) {
					command.shutdown();
					interruptedCommands.add(command);
				}
				activeCommands.clear();
			}

		}

		Iterator<Command> commands = activeCommands.iterator();




		ArrayList<Command> nextCommands = new ArrayList<>();
		while (commands.hasNext()) {
			Command command = commands.next();
			command.periodic();

			if (command.completed()) {
				command.shutdown();
				if (command.getNext() != null) {
					nextCommands.add(command.getNext());
				} else if (command.canInterruptOthers) {
					// restart all interrupted commands
					for (Command interruptedCommand : interruptedCommands) {
						interruptedCommand.init();
						activeCommands.add(interruptedCommand);
					}

				}

				commands.remove();

			}
		}

		for (Command nextCommand : nextCommands)
			forceCommand(nextCommand);
	}

	public void forceCommand(Command command) {
		ArrayList<Subsystem> nextCommandDependencies = command.getDependencies();

		Iterator<Command> currentCommands = activeCommands.iterator();

		while (currentCommands.hasNext()) {
			Command currentCommand = currentCommands.next();
			for (Subsystem subsystem : currentCommand.getDependencies())
				if (nextCommandDependencies.contains(subsystem)) {
					currentCommand.shutdown();
					currentCommands.remove();
					break;
				}
		}

		activeCommands.add(command);
		command.init();
	}

	public boolean isEmpty() {
		return activeCommands.isEmpty();
	}

	public void appendSubsystem(Subsystem s) {
		subsystems.add(s);
	}
}
