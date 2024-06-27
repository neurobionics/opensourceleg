"""
Volitional walking controller for the OSL.
This implementation is all in python.

Ryan Posh, Jonathan Tittle
Department of Aerospace and Mechanical Engineering
University of Notre Dame

Date Started: 3/20/24
Last Edits: 5/07/24
"""
import numpy as np
import opensourceleg.tools.units as units
from opensourceleg.osl import OpenSourceLeg
from OSL_EMG_Functions import EMG

offline_mode = False  # Set to true for debugging without hardware

ankle_stiffness = 200
ankle_damping = 5
passive_ankle_position = 0
subjectID = 'Ryan'
cal_filename = './' + subjectID + '_EMG_Cal.yaml' # discuss path with UM team

class VolitionalController:
    def __init__(self, emg):
        self.emg = emg
        pass
    
    def update(self):
        emg_GAS = emg.update('GAS')
        emg_TA = emg.update('TA')
        u = self.decode(emg_GAS, emg_TA, emg.calEMGData) # call volitional input function
        return u, emg_GAS, emg_TA

    def decode(self, emg_avg_1, emg_avg_2, calibration_parameters):
        """
        Converts from raw EMG values to volitional control input
        """
        if emg_avg_2 > 2*calibration_parameters.stdev_2 or emg_avg_1 > 2*calibration_parameters.stdev_1:
            # if signals are more significant than baseline noise:

            u_p = emg_avg_2/calibration_parameters.MVA_GAS
            u_d = emg_avg_1/calibration_parameters.MVA_TA

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
        else:
            u = 0
            print("None")
        return u

if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200)
    # osl.add_joint(name="knee", gear_ratio=41.4999, offline_mode=offline_mode)
    osl.add_joint(name="ankle", gear_ratio=41.4999, offline_mode=offline_mode)
 
    osl.log.add_attributes(container=osl, attributes=["timestamp"])
    # osl.log.add_attributes(
    #     container=osl.knee,
    #     attributes=[
    #         "output_position",
    #         "motor_current",
    #         "joint_torque",
    #         "motor_voltage",
    #         "accelx",
    #     ],
    # )
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

    emg = EMG(a_channel = 2, b_channel = 3, time_window = .1, time_step = 0.001, ADC_offset_a=510, ADC_offset_b=515)
    emg.calEMGLoad(cal_filename)

    volitional_controller = VolitionalController(emg)
    
    with osl:
        osl.home()

        for t in osl.clock:
            osl.update()
            u, _, _ = volitional_controller.update()

            if osl._has_knee == True and osl._has_ankle == True:
                
                gamma = time_step*(max_knee_velocity)

                theta = osl.knee.output_position + gamma*u   

                if osl.knee.mode != osl.knee.control_modes.impedance:
                    osl.knee.set_mode(mode=osl.knee.control_modes.impedance)
                    osl.knee.set_impedance_gains()
                osl.knee.set_joint_impedance(
                    K=units.convert_to_default(
                        stiffness,
                        units.stiffness.N_m_per_rad,
                    ),
                    B=units.convert_to_default(
                        damping,
                        units.damping.N_m_per_rad_per_s,
                    ),
                )
                osl.knee.set_output_position(
                    position=units.convert_to_default(
                        theta, units.position.deg
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

                gamma = 20 #? (ROM) should be ROM/2, for max plantar/dorsi angle
                theta = gamma*u

                if osl.ankle.mode != osl.ankle.control_modes.impedance:
                    osl.ankle.set_mode(osl.ankle.control_modes.impedance)
                    osl.ankle.set_impedance_gains()
                osl.ankle.set_joint_impedance(
                    K=units.convert_to_default(
                        ankle_stiffness,
                        units.stiffness.N_m_per_rad,
                    ),
                    B=units.convert_to_default(
                        ankle_damping,
                        units.damping.N_m_per_rad_per_s,
                    ),
                )
                osl.ankle.set_output_position(
                    position=units.convert_to_default(
                        theta, units.position.deg
                    ),
                )

            else: 
                print('Configuration issue. Please check that either the knee and ankle or the ankle only are configured.')
 
            print(
                "Theta {:.2f}, u {:.2f}, Ankle Angle (d) {:.2f}".format(
                    theta, u, units.convert_from_default(osl.ankle.output_position, units.position.deg),
                ),
                end="\r",
            )
