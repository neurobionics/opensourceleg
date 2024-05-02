"""
Volitional walking controller for the OSL.
This implementation is all in python.

Ryan Posh, Jonathan Tittle
Department of Aerospace and Mechanical Engineering
University of Notre Dame

Date Started: 3/20/24
Last Edits: 4/09/24
"""
import numpy as np
import opensourceleg.tools.units as units
from opensourceleg.control.state_machine import Event, State, StateMachine
from opensourceleg.osl import OpenSourceLeg
from opensourceleg.control import OSL_Functions_EMG as emg

offline_mode = False  # Set to true for debugging without hardware

passive_stiffness = 0.08
passive_damping = 0.1
passive_ankle_position = 0

# Prompt user for body weight [in kg]
BODY_WEIGHT = int(input('Please input the subject weight in [kg]: '))
# Device Configuration (Uncomment the desired configuration and comment out the undesired configuration)
CONFIG = knee_ankle
#CONFIG = ankle_only



#-------------------- Setup-------------------- 
# Create Class Object for Actuator Commands
FX = fx.FlexSEA()
# Open Device ID and Start Streaming
devId = opcl.devOpen(FX) 
# NEED TO UPDATE ALL devId STUFF TO BE UMICH COMPATIBLE

cal_filename = 'SubjectDetails/' + subjectID + '_EMG_Cal.yaml' # discuss path with UM team

calEMGData = emg.CalEMGDataSingle()
time_window = 0.1
freq_delay = 1/1000
window = int(time_window/freq_delay)
reading_1 = np.zeros(window)
reading_2 = np.zeros(window)
calEMGData = emg.emgCalibration(FX, calEMGData, time_window, freq_delay, cal_filename)


def run_volitional_controller():
    """
    This is the main function for this script.
    It creates an OSL object and builds a volitional controller.
    It runs a main loop that updates volitional input based on the
    EMG input and sends updated commands to the motors.
    """
    osl = OpenSourceLeg(frequency=200)
    osl.add_joint(name="knee", gear_ratio=41.4999, offline_mode=offline_mode)
    osl.add_joint(name="ankle", gear_ratio=41.4999, offline_mode=offline_mode)
    LOADCELL_MATRIX = np.array(
        [
            (-38.72600, -1817.74700, 9.84900, 43.37400, -44.54000, 1824.67000),
            (-8.61600, 1041.14900, 18.86100, -2098.82200, 31.79400, 1058.6230),
            (-1047.16800, 8.63900, -1047.28200, -20.70000, -1073.08800, -8.92300),
            (20.57600, -0.04000, -0.24600, 0.55400, -21.40800, -0.47600),
            (-12.13400, -1.10800, 24.36100, 0.02300, -12.14100, 0.79200),
            (-0.65100, -28.28700, 0.02200, -25.23000, 0.47300, -27.3070),
        ]
    )
    osl.add_loadcell(
        dephy_mode=False,
        offline_mode=offline_mode,
        loadcell_matrix=LOADCELL_MATRIX,
    )

    # Create Class Object for SPI Functions Related to Force/EMG Sensors
    #sleep(5*osl.dtCenti)
    #spi = spidev.SpiDev()
    #spi.open(0, 0)
    #spi.max_speed_hz=1000000

    spi = emg.initialize_spi()

    volitional = volitional_decoder(osl) # Build volitional class
    #passive_ankle = passive_impedance(osl) # Build Passive impedance function that sets the gains for passive ankle 

    osl.log.add_attributes(container=osl, attributes=["timestamp"])
    osl.log.add_attributes(
        container=osl.knee,
        attributes=[
            "output_position",
            "motor_current",
            "joint_torque",
            "motor_voltage",
            "accelx",
        ],
    )
    osl.log.add_attributes(
        container=osl.ankle,
        attributes=[
            "output_position",
            "motor_current",
            "joint_torque",
            "motor_voltage",
            "accelx",
        ],
    )
    osl.log.add_attributes(container=osl.loadcell, attributes=["fz"])

    # Prompt the user on whether or not they want to conduct a new calibration.
    calChoice = int(input('Use existing calibration data for Motor Range or run calibration sequence (1 for Existing, 2 for New Data): '))
    calData = calibration(calChoice)
	# Update to be consistent with UM team
    
    with osl:
        osl.home()
        volitional.start() 
        #passive_ankle.start() 

        for t in osl.clock:
            osl.update()
            volitional.update() 
            #passive_ankle.update() 
    
            raw_EMG_1 = emg.readadc(osl.pChan2, spi) # Read in Raw EMG for muscle 1
            raw_EMG_2 = emg.readadc(osl.pChan3, spi) # Read in Raw EMG for muscle 2

            u = emg.voltional_decoder(raw_EMG_1, raw_EMG_2, calEMGData) # call volitional input function

            # This block is setting the transitions and impedance values and eq position as needed for knee:

            if osl._has_knee == True and osl._has_ankle == True:

                volitional.gamma = X*(maximum velocity) ######### Need to decide max vel or allow it to be a tuned parameter

                volitional.joint_parameters.theta = osl.knee.output_position + volitional.gamma*u   

                if osl.knee.mode != osl.knee.control_modes.impedance:
                    osl.knee.set_mode(mode=osl.knee.control_modes.impedance)
                    osl.knee.set_impedance_gains()
                osl.knee.set_joint_impedance(
                    K=units.convert_to_default(
                        volitional.joint_parameters.stiffness,
                        units.stiffness.N_m_per_rad,
                    ),
                    B=units.convert_to_default(
                        volitional.joint_parameters.damping,
                        units.damping.N_m_per_rad_per_s,
                    ),
                )
                osl.knee.set_output_position(
                    position=units.convert_to_default(
                        volitional.joint_parameters.theta, units.position.deg
                    ),
                )

                if osl.ankle.mode != osl.ankle.control_modes.impedance:
                    osl.ankle.set_mode(osl.ankle.control_modes.impedance)
                    osl.ankle.set_impedance_gains()
                osl.ankle.set_joint_impedance(
                    K=units.convert_to_default(
                        passive_stiffness,
                        units.stiffness.N_m_per_rad,
                    ),
                    B=units.convert_to_default(
                        passive_damping,
                        units.damping.N_m_per_rad_per_s,
                    ),
                )
                osl.ankle.set_output_position(
                    position=units.convert_to_default(
                        passive_ankle_position, units.position.deg
                    ),
                )


            elif osl._has_ankle == True and osl._has_knee == False:

                volitional.gamma = X (ROM)

                volitional.joint_parameters.theta = volitional.gamma*u

                if osl.ankle.mode != osl.ankle.control_modes.impedance:
                    osl.ankle.set_mode(osl.ankle.control_modes.impedance)
                    osl.ankle.set_impedance_gains()
                osl.ankle.set_joint_impedance(
                    K=units.convert_to_default(
                        volitional.joint_parameters.stiffness,
                        units.stiffness.N_m_per_rad,
                    ),
                    B=units.convert_to_default(
                        volitional.joint_parameters.damping,
                        units.damping.N_m_per_rad_per_s,
                    ),
                )
                osl.ankle.set_output_position(
                    position=units.convert_to_default(
                        volitional.joint_parameters.theta, units.position.deg
                    ),
                )

	    else: 
		print('Configuration issue. Please check that either the knee and ankle or the ankle only are configured.')
 
            print(
                "Current time in state {}: {:.2f} seconds, Knee Eq {:.2f}, Ankle Eq {:.2f}, Fz {:.2f}".format(
                    #fsm.current_state.name,
                    #fsm.current_state.current_time_in_state,
                    #fsm.current_state.knee_theta,
                    #fsm.current_state.ankle_theta,
                    # ADD OUR THINGS TO PRINT HERE **************************************
                    osl.loadcell.fz,
                ),
                end="\r",
            )
        volitional.stop() 
        print("")

def volitional_decoder(raw_EMG_1, raw_EMG_2, calibration_parameters):
    """
    Converts from raw EMG values to volitional control input
    """

    # rectify and smooth the EMG:
    emg_raw_rect_1 = emg.rectify_emg(raw_EMG_1, calibration_parameters.baseline_1)
    emg_avg_1, reading_1 = emg.filter_emg(emg_raw_rect_1, time_window, freq_delay, reading_1)

    emg_raw_rect_2 = emg.rectify_emg(raw_EMG_2, calibration_parameters.baseline_2)
    emg_avg_2, reading_2 = emg.filter_emg(emg_raw_rect_2, time_window, freq_delay, reading_2)

    if emg_avg_3 > 2*calibration_parameters.stdev_3 or emg_avg_2 > 2*calibration_parameters.stdev_2:
        # if signals are more significant than baseline noise:

        u_p = emg_avg_3/calibration_parameters.MVA_GAS
        u_d = emg_avg_2/calibration_parameters.MVA_TA

            
        if u_p > 1:
                u_p = 1
        if u_d > 1:
                u_d = 1

        # calculate the slope and magnitude of the new point
        m = abs((u_p)/(u_d))

        if m > calibration_parameters.m_gas:
            m = calibration_parameters.m_gas
        if m < calibration_parameters.m_ta:
            m = calibration_parameters.m_ta
            
        K_0 = np.sqrt(u_p*u_p + u_d*u_d)

        if K_0 > 1:
                K_0 = 1    # I am pretty sure that is right

        # determine if it is a desired plantarflex or dorsiflex
        if m > calibration_parameters.m_0: # plantarflex
            # project the point onto the plantarflextion vector
            u = K_0*(m - calibration_parameters.m_0)/(calibration_parameters.m_gas - calibration_parameters.m_0)

        if m < calibration_parameters.m_0: # dorsiflex
            # project the point onto the dorsiflexion vector
            u = -K_0*(m - calibration_parameters.m_0)/(calibration_parameters.m_ta - calibration_parameters.m_0)

     
    #self.joint_parameters.stiffness = XX
    #self.joint_parameters.damping = XX
    #self.joint_parameters.theta = XX


    return u

#def passive_impedance(theta, k, b):
   # Object that defines the passive ankle and sets its impedance parameters 

   #self.joint_parameters.stiffness = k
   #self.joint_parameters.damping = b
   #self.joint_parameters.theta = theta

# This next function is probably already taken care of by UM team in some way?
def calibration(calChoice):
   if calChoice == 1:

       # Load Calibration Data Previously Stored in Ankle_Cal.yaml
       calData = stor.calLoad(devId)
       print('Data Loaded Successfully')
       ###### May have to hard code in loading the stored file

   else:

       # Use as Catch All to Fall Back on Rerunning Calibration as a Precaution
       if calChoice != 2:

           print('Invalid option chosen, running calibration sequence as back up...')

       # Create Class Object for Calibration Data, Run Calibration Sequence
       calData = pac.CalDataSingle()
       calData = pac.ankleCalMot(devId, FX, calData, cal=1) ##############MAY NEED TO UN"PAC" THIS DATA

       print('Calibration successful...')

   return calData

if __name__ == "__main__":
    run_volitional_controller()
