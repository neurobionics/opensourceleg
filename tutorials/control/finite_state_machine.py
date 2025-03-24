from opensourceleg.control.fsm import State, StateMachine
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.logging.logger import Logger

# Transition criteria functions
def cleaning_to_docking(battery_level: float) -> bool:
    """
    Transition from 'Cleaning' to 'Docking' when the battery level is below 20%.
    """
    return battery_level < 20

def charging_to_cleaning(battery_level: float) -> bool:
    """
    Transition from 'Charging' to 'Cleaning' when the battery is fully charged (100%).
    """
    return battery_level == 100

# Main function
if __name__ == "__main__":
    # Initialize the logger for the finite state machine
    fsm_example_logger = Logger(
        log_path="./logs",  # Directory to store logs
        file_name="fsm.log",  # Log file name
    )

    # Define states
    cleaning_s = State(name="Cleaning")  # State for cleaning
    charging_s = State(name="Charging")  # State for charging
    docking_s = State(name="Docking", minimum_time_in_state=10)  # State for docking, assuming it takes 10 seconds

    # Initialize the state machine with the defined states and the initial state
    fsm = StateMachine(states=[charging_s, cleaning_s, docking_s], initial_state_name=charging_s.name)

    # Add transitions between states
    fsm.add_transition(
        source=charging_s,
        destination=cleaning_s,
        event_name="Fully Charged - Beginning Cleaning",
        criteria=charging_to_cleaning
    )
    fsm.add_transition(
        source=cleaning_s,
        destination=docking_s,
        event_name="Battery Low - Finding Charging Dock",
        criteria=cleaning_to_docking
    )
    fsm.add_transition(
        source=docking_s,
        destination=charging_s,
        event_name="Docked - Beginning Charging"
    )

    # Initialize the battery level
    battery_level = 50.0  # Start with 50% battery

    # Run the state machine in a soft real-time loop
    with fsm:
        for t in SoftRealtimeLoop(dt=0.5):  # Loop with a time step of 0.5 seconds

            # Simulate battery behavior based on the current state
            if fsm.current_state == charging_s:
                # Battery charges by 2% per loop iteration
                battery_level += 2.0
            elif fsm.current_state == cleaning_s:
                # Battery drains by 5% per loop iteration during cleaning
                battery_level -= 5.0
            else:
                # Battery drains slowly (0.5%) while docking
                battery_level -= 0.5

            # Clamp the battery level between 0% and 100%
            battery_level = min(100, max(battery_level, 0))

            # Update the state machine with the current battery level
            fsm.update(battery_level=battery_level)

            # Log the current state and battery level
            fsm_example_logger.info(
                f"Current state: {fsm.current_state.name}; "
                f"Battery level: {battery_level}; "
            )
