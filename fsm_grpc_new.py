import numpy as np

# from grpc_server import GRPCManager
import opensourceleg.tools.units as units
from opensourceleg.control.state_machine import Event, State, StateMachine
from opensourceleg.osl import OpenSourceLeg
import time
import grpc
from concurrent import futures
import time
import math
import os
from google.protobuf import descriptor_pb2
from google.protobuf.compiler import plugin_pb2
from grpc_tools import protoc
import importlib

offline_mode = False  # Set to true for debugging without hardware

# ------------- TUNABLE FSM PARAMETERS ---------------- #
BODY_WEIGHT = 60 * 9.8



class GRPCManager:
    def __init__(self, port=50051, frequency=200):
        self.server_data = {}
        self.client_data = {}
        self.callbacks = {}
        self.proto_file = "dynamic_message.proto"
        self.pb2_file = "dynamic_message_pb2.py"
        self.pb2_grpc_file = "dynamic_message_pb2_grpc.py"
        self.server = None
        self.stub = None
        self.port = port
        self.pb2 = None
        self.pb2_grpc = None
        self.frequency = frequency

        self.KNEE_K_ESTANCE = 200.0
        self.KNEE_B_ESTANCE = 5.0
        self.KNEE_THETA_ESTANCE = 5.0
        self.ANKLE_K_ESTANCE = 100.0
        self.ANKLE_B_ESTANCE = 3.0
        self.ANKLE_THETA_ESTANCE = -2
        self.LOAD_LSTANCE = -1.0 * BODY_WEIGHT * 0.25
        self.ANKLE_THETA_ESTANCE_TO_LSTANCE = np.deg2rad(6.0)

        self.KNEE_K_LSTANCE = 99.372
        self.KNEE_B_LSTANCE = 1.272
        self.KNEE_THETA_LSTANCE = 8
        self.ANKLE_K_LSTANCE = 135
        self.ANKLE_B_LSTANCE = 0.063
        self.ANKLE_THETA_LSTANCE = -20
        self.LOAD_ESWING: float = -1.0 * BODY_WEIGHT * 0.15

        self.KNEE_K_ESWING = 30
        self.KNEE_B_ESWING = 1.0
        self.KNEE_THETA_ESWING = 50
        self.ANKLE_K_ESWING = 25.0
        self.ANKLE_B_ESWING = 1.0
        self.ANKLE_THETA_ESWING = 25
        self.KNEE_THETA_ESWING_TO_LSWING = np.deg2rad(20)
        self.KNEE_DTHETA_ESWING_TO_LSWING = -1.0

        self.KNEE_K_LSWING = 55
        self.KNEE_B_LSWING = 3
        self.KNEE_THETA_LSWING = 5
        self.ANKLE_K_LSWING = 30.0
        self.ANKLE_B_LSWING = 5.0
        self.ANKLE_THETA_LSWING = 5
        self.LOAD_ESTANCE: float = -1.0 * BODY_WEIGHT * 0.15
        self.KNEE_THETA_LSWING_TO_ESTANCE = np.deg2rad(30)

    def add_variable(self, name, getter_func):
        """Add a variable to be sent over gRPC."""
        self.server_data[name] = getter_func

    def add_callback(self, name, callback_func):
        """Add a callback function to be called from the client."""
        self.callbacks[name] = callback_func

    def _generate_proto_file(self):
        """Generate the proto file based on added variables."""
        with open(self.proto_file, "w") as f:
            f.write('syntax = "proto3";\n\n')
            f.write("package dynamic;\n\n")
            f.write("service DynamicService {\n")
            f.write("  rpc SendData (Data) returns (Empty) {}\n")
            f.write("  rpc ReceiveData (Empty) returns (stream Data) {}\n")
            f.write("  rpc CallFunction (Function) returns (Empty) {}\n")
            f.write("}\n\n")
            f.write("message Data {\n")
            f.write("  map<string, float> values = 1;\n")
            f.write("}\n\n")
            f.write("message Function {\n")
            f.write("  string name = 1;\n")
            f.write("}\n\n")
            f.write("message Empty {}\n")

    def _compile_proto_file(self):
        """Compile the proto file to generate pb2 and pb2_grpc files."""
        protoc.main(
            ("", f"-I.", f"--python_out=.", f"--grpc_python_out=.", self.proto_file)
        )

    def _import_generated_modules(self):
        """Import the dynamically generated modules."""
        self.pb2 = importlib.import_module(self.pb2_file.split(".")[0])
        self.pb2_grpc = importlib.import_module(self.pb2_grpc_file.split(".")[0])

    def start(self):
        """Generate proto file, compile it, and start the gRPC server."""
        self._generate_proto_file()
        self._compile_proto_file()
        self._import_generated_modules()

        class DynamicService(self.pb2_grpc.DynamicServiceServicer):
            def __init__(self, manager):
                self.manager = manager

            def SendData(self, request, context):
                self.manager.client_data = request.values
                print("Data received:", self.manager.client_data)
                return self.manager.pb2.Empty()

            def ReceiveData(self, request, context):
                while True:
                    _data = self.manager.pb2.Data(
                        values={
                            name: float(getter())
                            for name, getter in self.manager.server_data.items()
                        }
                    )
                    yield _data
                    time.sleep(1 / self.manager.frequency)

            def CallFunction(self, request, context):
                if request.name in self.manager.callbacks:
                    self.manager.callbacks[request.name]()
                else:
                    print(f"Function {request.name} not found")

                return self.manager.pb2.Empty()

        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        self.pb2_grpc.add_DynamicServiceServicer_to_server(
            DynamicService(self), self.server
        )
        self.server.add_insecure_port(f"[::]:{self.port}")
        self.server.start()
        print(f"gRPC Server started on port {self.port}")

    def update(self):
        """Send an update message to the server with current variable values."""
        if not self.stub:
            channel = grpc.insecure_channel(f"localhost:{self.port}")
            self.stub = self.pb2_grpc.DynamicServiceStub(channel)

    def stop(self):
        """Stop the gRPC server."""
        if self.server:
            self.server.stop(0)
            print("gRPC Server stopped")

    def update_values(self):
        if len(self.client_data.keys()) > 0:
            self.KNEE_K_ESTANCE = self.client_data["KNEE_K_ESTANCE"]
            self.KNEE_B_ESTANCE = self.client_data["KNEE_B_ESTANCE"]
            self.KNEE_THETA_ESTANCE = self.client_data["KNEE_THETA_ESTANCE"]
            self.ANKLE_K_ESTANCE = self.client_data["ANKLE_K_ESTANCE"]
            self.ANKLE_B_ESTANCE = self.client_data["ANKLE_B_ESTANCE"]
            self.ANKLE_THETA_ESTANCE = self.client_data["ANKLE_THETA_ESTANCE"]
            self.LOAD_LSTANCE = (
                -1.0 * BODY_WEIGHT * self.client_data["LOAD_LSTANCE"]
            )
            self.ANKLE_THETA_ESTANCE_TO_LSTANCE = np.deg2rad(
                self.client_data["ANKLE_THETA_ESTANCE_TO_LSTANCE"]
            )

            self.KNEE_K_LSTANCE = self.client_data["KNEE_K_LSTANCE"]
            self.KNEE_B_LSTANCE = self.client_data["KNEE_B_LSTANCE"]
            self.KNEE_THETA_LSTANCE = self.client_data["KNEE_THETA_LSTANCE"]
            self.ANKLE_K_LSTANCE = self.client_data["ANKLE_K_LSTANCE"]
            self.ANKLE_B_LSTANCE = self.client_data["ANKLE_B_LSTANCE"]
            self.ANKLE_THETA_LSTANCE = self.client_data["ANKLE_THETA_LSTANCE"]
            self.LOAD_ESWING = (
                -1.0 * BODY_WEIGHT * self.client_data["LOAD_ESWING"]
            )

            self.KNEE_K_ESWING = self.client_data["KNEE_K_ESWING"]
            self.KNEE_B_ESWING = self.client_data["KNEE_B_ESWING"]
            self.KNEE_THETA_ESWING = self.client_data["KNEE_THETA_ESWING"]
            self.ANKLE_K_ESWING = self.client_data["ANKLE_K_ESWING"]
            self.ANKLE_B_ESWING = self.client_data["ANKLE_B_ESWING"]
            self.ANKLE_THETA_ESWING = self.client_data["ANKLE_THETA_ESWING"]
            self.KNEE_THETA_ESWING_TO_LSWING = np.deg2rad(
                self.client_data["KNEE_THETA_ESWING_TO_LSWING"]
            )
            self.KNEE_DTHETA_ESWING_TO_LSWING = self.client_data[
                "KNEE_DTHETA_ESWING_TO_LSWING"
            ]

            self.KNEE_K_LSWING = self.client_data["KNEE_K_LSWING"]
            self.KNEE_B_LSWING = self.client_data["KNEE_B_LSWING"]
            self.KNEE_THETA_LSWING = self.client_data["KNEE_THETA_LSWING"]
            self.ANKLE_K_LSWING = self.client_data["ANKLE_K_LSWING"]
            self.ANKLE_B_LSWING = self.client_data["ANKLE_B_LSWING"]
            self.ANKLE_THETA_LSWING = self.client_data["ANKLE_THETA_LSWING"]
            self.LOAD_ESTANCE = (
                -1.0 * BODY_WEIGHT * self.client_data["LOAD_ESTANCE"]
            )
            self.KNEE_THETA_LSWING_TO_ESTANCE = np.deg2rad(
                self.client_data["KNEE_THETA_LSWING_TO_ESTANCE"]
            )

GRPCMANAGER = GRPCManager(port=50051)

# ---------------------------------------------------- #


def update_fsm_values(fsm):
    fsm._states[1].set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_ESTANCE,
        k=GRPCMANAGER.KNEE_K_ESTANCE,
        b=GRPCMANAGER.KNEE_B_ESTANCE,
    )
    fsm._states[1].set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_ESTANCE,
        k=GRPCMANAGER.ANKLE_K_ESTANCE,
        b=GRPCMANAGER.ANKLE_B_ESTANCE,
    )

    fsm._states[2].set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_LSTANCE,
        k=GRPCMANAGER.KNEE_K_LSTANCE,
        b=GRPCMANAGER.KNEE_B_LSTANCE,
    )

    fsm._states[2].set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_LSTANCE,
        k=GRPCMANAGER.ANKLE_K_LSTANCE,
        b=GRPCMANAGER.ANKLE_B_LSTANCE,
    )

    fsm._states[3].set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_ESWING,
        k=GRPCMANAGER.KNEE_K_ESWING,
        b=GRPCMANAGER.KNEE_B_ESWING,
    )

    fsm._states[3].set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_ESWING,
        k=GRPCMANAGER.ANKLE_K_ESWING,
        b=GRPCMANAGER.ANKLE_B_ESWING,
    )

    fsm._states[4].set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_LSWING,
        k=GRPCMANAGER.KNEE_K_LSWING,
        b=GRPCMANAGER.KNEE_B_LSWING,
    )

    fsm._states[4].set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_LSWING,
        k=GRPCMANAGER.ANKLE_K_LSWING,
        b=GRPCMANAGER.ANKLE_B_LSWING,
    )


def get_state_value(state_name):
    if state_name == "e_stance":
        return 1
    elif state_name == "l_stance":
        return 2
    elif state_name == "e_swing":
        return 3
    elif state_name == "l_swing":
        return 4
    else:
        return 0


def run_FSM_controller():
    """
    This is the main function for this script.
    It creates an OSL object and builds a state machine with 4 states.
    It runs a main loop that updates the state machine based on the
    hardware information and sends updated commands to the motors.
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

    fsm = build_4_state_FSM(osl)

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
    osl.log.add_attributes(container=fsm.current_state, attributes=["name"])

    state_value = get_state_value(fsm.current_state.name)

    GRPCMANAGER.add_variable("knee_output_position", lambda: osl.knee.output_position)
    GRPCMANAGER.add_variable("ankle_output_position", lambda: osl.ankle.output_position)
    GRPCMANAGER.add_variable("state", lambda: state_value)
    # GRPCMANAGER.add_variable("loadcell_fz", lambda: osl.loadcell.fz)

    GRPCMANAGER.start()

    with osl:
        osl.home()
        fsm.start()

        for t in osl.clock:
            osl.update()
            fsm.update()

            state_value = get_state_value(fsm.current_state.name)

            GRPCMANAGER.update_values()
            update_fsm_values(fsm)

            if osl.knee.mode != osl.knee.control_modes.impedance:
                osl.knee.set_mode(mode=osl.knee.control_modes.impedance)
                osl.knee.set_impedance_gains()
            osl.knee.set_joint_impedance(
                K=units.convert_to_default(
                    fsm.current_state.knee_stiffness,
                    units.stiffness.N_m_per_rad,
                ),
                B=units.convert_to_default(
                    fsm.current_state.knee_damping,
                    units.damping.N_m_per_rad_per_s,
                ),
            )
            osl.knee.set_output_position(
                position=units.convert_to_default(
                    fsm.current_state.knee_theta, units.position.deg
                ),
            )

            if osl.ankle.mode != osl.ankle.control_modes.impedance:
                osl.ankle.set_mode(osl.ankle.control_modes.impedance)
                osl.ankle.set_impedance_gains()
            osl.ankle.set_joint_impedance(
                K=units.convert_to_default(
                    fsm.current_state.ankle_stiffness,
                    units.stiffness.N_m_per_rad,
                ),
                B=units.convert_to_default(
                    fsm.current_state.ankle_damping,
                    units.damping.N_m_per_rad_per_s,
                ),
            )
            osl.ankle.set_output_position(
                position=units.convert_to_default(
                    fsm.current_state.ankle_theta, units.position.deg
                ),
            )
            print(
                "Current time in state {}: {:.2f} seconds, Knee Eq {:.2f}, Ankle Eq {:.2f}, K Stiffness {:.2f}, A Stiffness {:.2f}, K Damping {:.2f}, A Damping {:.2f}, Fz {:.2f}".format(
                    fsm.current_state.name,
                    fsm.current_state.current_time_in_state,
                    fsm.current_state.knee_theta,
                    fsm.current_state.ankle_theta,
                    fsm.current_state.knee_stiffness,
                    fsm.current_state.ankle_stiffness,
                    fsm.current_state.knee_damping,
                    fsm.current_state.ankle_damping,
                    osl.loadcell.fz,
                ),
                end="\r",
            )

        GRPCMANAGER.stop()
        fsm.stop()
        print("")


def build_4_state_FSM(osl: OpenSourceLeg) -> StateMachine:
    """This method builds a state machine with 4 states.
    The states are early stance, late stance, early swing, and late swing.
    It uses the impedance parameters and transition criteria above.

    Inputs:
        OSL instance
    Returns:
        FSM object"""

    early_stance = State(name="e_stance")
    late_stance = State(name="l_stance")
    early_swing = State(name="e_swing")
    late_swing = State(name="l_swing")

    early_stance.set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_ESTANCE,
        k=GRPCMANAGER.KNEE_K_ESTANCE,
        b=GRPCMANAGER.KNEE_B_ESTANCE,
    )
    early_stance.make_knee_active()
    early_stance.set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_ESTANCE,
        k=GRPCMANAGER.ANKLE_K_ESTANCE,
        b=GRPCMANAGER.ANKLE_B_ESTANCE,
    )
    early_stance.make_ankle_active()

    late_stance.set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_LSTANCE,
        k=GRPCMANAGER.KNEE_K_LSTANCE,
        b=GRPCMANAGER.KNEE_B_LSTANCE,
    )
    late_stance.make_knee_active()
    late_stance.set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_LSTANCE,
        k=GRPCMANAGER.ANKLE_K_LSTANCE,
        b=GRPCMANAGER.ANKLE_B_LSTANCE,
    )
    late_stance.make_ankle_active()

    early_swing.set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_ESWING,
        k=GRPCMANAGER.KNEE_K_ESWING,
        b=GRPCMANAGER.KNEE_B_ESWING,
    )
    early_swing.make_knee_active()
    early_swing.set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_ESWING,
        k=GRPCMANAGER.ANKLE_K_ESWING,
        b=GRPCMANAGER.ANKLE_B_ESWING,
    )
    early_swing.make_ankle_active()

    late_swing.set_knee_impedance_paramters(
        theta=GRPCMANAGER.KNEE_THETA_LSWING,
        k=GRPCMANAGER.KNEE_K_LSWING,
        b=GRPCMANAGER.KNEE_B_LSWING,
    )
    late_swing.make_knee_active()
    late_swing.set_ankle_impedance_paramters(
        theta=GRPCMANAGER.ANKLE_THETA_LSWING,
        k=GRPCMANAGER.ANKLE_K_LSWING,
        b=GRPCMANAGER.ANKLE_B_LSWING,
    )
    late_swing.make_ankle_active()

    def estance_to_lstance(osl: OpenSourceLeg) -> bool:
        """
        Transition from early stance to late stance when the loadcell
        reads a force greater than a threshold.
        """
        assert osl.loadcell is not None
        return bool(
            osl.loadcell.fz < GRPCMANAGER.LOAD_LSTANCE
            and osl.ankle.output_position > GRPCMANAGER.ANKLE_THETA_ESTANCE_TO_LSTANCE
        )

    def lstance_to_eswing(osl: OpenSourceLeg) -> bool:
        """
        Transition from late stance to early swing when the loadcell
        reads a force less than a threshold.
        """
        assert osl.loadcell is not None
        return bool(osl.loadcell.fz > GRPCMANAGER.LOAD_ESWING)

    def eswing_to_lswing(osl: OpenSourceLeg) -> bool:
        """
        Transition from early swing to late swing when the knee angle
        is greater than a threshold and the knee velocity is less than
        a threshold.
        """
        assert osl.knee is not None
        return bool(
            osl.knee.output_position > GRPCMANAGER.KNEE_THETA_ESWING_TO_LSWING
            and osl.knee.output_velocity < GRPCMANAGER.KNEE_DTHETA_ESWING_TO_LSWING
        )

    def lswing_to_estance(osl: OpenSourceLeg) -> bool:
        """
        Transition from late swing to early stance when the loadcell
        reads a force greater than a threshold or the knee angle is
        less than a threshold.
        """
        assert osl.knee is not None and osl.loadcell is not None
        return bool(
            osl.loadcell.fz
            < GRPCMANAGER.LOAD_ESTANCE
            # or osl.knee.output_position < GRPCMANAGER.KNEE_THETA_LSWING_TO_ESTANCE
        )

    foot_flat = Event(name="foot_flat")
    heel_off = Event(name="heel_off")
    toe_off = Event(name="toe_off")
    pre_heel_strike = Event(name="pre_heel_strike")
    heel_strike = Event(name="heel_strike")

    fsm = StateMachine(osl=osl, spoof=offline_mode)
    fsm.add_state(state=early_stance, initial_state=True)
    fsm.add_state(state=late_stance)
    fsm.add_state(state=early_swing)
    fsm.add_state(state=late_swing)

    fsm.add_event(event=foot_flat)
    fsm.add_event(event=heel_off)
    fsm.add_event(event=toe_off)
    fsm.add_event(event=pre_heel_strike)
    fsm.add_event(event=heel_strike)

    fsm.add_transition(
        source=early_stance,
        destination=late_stance,
        event=foot_flat,
        callback=estance_to_lstance,
    )
    fsm.add_transition(
        source=late_stance,
        destination=early_swing,
        event=heel_off,
        callback=lstance_to_eswing,
    )
    fsm.add_transition(
        source=early_swing,
        destination=late_swing,
        event=toe_off,
        callback=eswing_to_lswing,
    )
    fsm.add_transition(
        source=late_swing,
        destination=early_stance,
        event=heel_strike,
        callback=lswing_to_estance,
    )
    return fsm


if __name__ == "__main__":
    run_FSM_controller()
