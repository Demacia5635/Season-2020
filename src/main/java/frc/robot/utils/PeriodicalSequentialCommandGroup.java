/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * Add your docs here.
 */
public class PeriodicalSequentialCommandGroup extends CommandGroupBaseDemacia {
  protected final List<Command> m_commands = new ArrayList<>();
  protected int m_currentCommandIndex = -1;
  protected boolean m_runWhenDisabled = true;

  /**
   * Creates a new SequentialCommandGroup.  The given commands will be run sequentially, with
   * the CommandGroup finishing when the last command finishes.
   *
   * @param commands the commands to include in this group.
   */
  public PeriodicalSequentialCommandGroup(final Command... commands) {
    addCommands(commands);
  }

  @Override
  public final void addCommands(final Command... commands) {
    requireUngrouped(commands);

    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a CommandGroup while the group is running");
    }

    registerGroupedCommands(commands);

    for (final Command command : commands) {
      m_commands.add(command);
      m_requirements.addAll(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
    }
  }

  @Override
  public void initialize() {
    m_currentCommandIndex = 0;

    if (!m_commands.isEmpty()) {
      m_commands.get(0).initialize();
    }
  }

  @Override
  public void execute() {
    if (m_commands.isEmpty()) {
      return;
    }

    final Command currentCommand;

    if(m_commands.size() == m_currentCommandIndex){
        m_currentCommandIndex = 0;
    }
    currentCommand = m_commands.get(m_currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      m_currentCommandIndex++;
      if (m_currentCommandIndex < m_commands.size()) {
        m_commands.get(m_currentCommandIndex).initialize();
      }
    }
  }

  @Override
  public void end(final boolean interrupted) {
    if (interrupted && !m_commands.isEmpty() && m_currentCommandIndex > -1
        && m_currentCommandIndex < m_commands.size()
    ) {
      m_commands.get(m_currentCommandIndex).end(true);
    }
    m_currentCommandIndex = -1;
  }

  @Override
  public boolean isFinished() {
    return m_currentCommandIndex == m_commands.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }
}
