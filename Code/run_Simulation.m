% Makeshift main for SimulationAlt - run to test SimulationAlt

simulation = SimulationAlt();
goalPosition = [0.5, 0.5, 1.5]; % Example goal position
simulation.moveWithVelocityKinematics(goalPosition);