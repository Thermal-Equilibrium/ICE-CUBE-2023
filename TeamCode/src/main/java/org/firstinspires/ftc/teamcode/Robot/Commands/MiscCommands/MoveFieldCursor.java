package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Purepursuit.Utils.VirtualField;

public class MoveFieldCursor extends Command {
	int x;
	int y;
	VirtualField virtualField;


	public MoveFieldCursor(int x, int y, VirtualField virtualField) {
		this.x = x;
		this.y = y;
		this.virtualField = virtualField;
	}

	@Override
	public void init() {
		virtualField.moveCursor(x, y);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return true;
	}

	@Override
	public void shutdown() {

	}
}
