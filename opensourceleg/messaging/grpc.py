import importlib
import math
import os
import time
from concurrent import futures

from opensourceleg.logging import LOGGER

try:
    import grpc
    from google.protobuf import descriptor_pb2
    from google.protobuf.compiler import plugin_pb2
    from grpc_tools import protoc

except ImportError:
    LOGGER.error("grpcio-tools, types-protobuf, and grpcio must be installed.")


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


def test_function():
    print("Test function called")


def estop():
    exit()


# Example usage
if __name__ == "__main__":
    manager = GRPCManager(port=50051)

    # Variables to be sent
    x = 0.0
    y = 0.0
    z = 0.0

    # Add variables using lambda functions
    manager.add_variable("x_value", lambda: math.sin(x))
    manager.add_variable("y_value", lambda: math.cos(y))
    manager.add_variable("z_value", lambda: z)

    manager.add_callback("estop", estop)

    # Start the server
    manager.start()

    try:
        t = 0.0
        # Simulate updates
        while True:
            x = t
            y = t
            z = t
            time.sleep(1 / 200)
            t += 1 / 200
    finally:
        manager.stop()
