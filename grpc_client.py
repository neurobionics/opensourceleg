import grpc
import time
import math
import dynamic_message_pb2
import dynamic_message_pb2_grpc


def run_client():
    channel = grpc.insecure_channel("localhost:50051")
    stub = dynamic_message_pb2_grpc.DynamicServiceStub(channel)

    # Send Data
    def send_data(
        KNEE_K_ESTANCE,
        KNEE_B_ESTANCE,
        KNEE_THETA_ESTANCE,
        ANKLE_K_ESTANCE,
        ANKLE_B_ESTANCE,
        ANKLE_THETA_ESTANCE,

        LOAD_LSTANCE,
        ANKLE_THETA_ESTANCE_TO_LSTANCE,


        KNEE_K_LSTANCE,
        KNEE_B_LSTANCE,
        KNEE_THETA_LSTANCE,
        ANKLE_K_LSTANCE,
        ANKLE_B_LSTANCE,
        ANKLE_THETA_LSTANCE,

        LOAD_ESWING,


        KNEE_K_ESWING,
        KNEE_B_ESWING,
        KNEE_THETA_ESWING,
        ANKLE_K_ESWING,
        ANKLE_B_ESWING,
        ANKLE_THETA_ESWING,

        KNEE_THETA_ESWING_TO_LSWING,
        KNEE_DTHETA_ESWING_TO_LSWING,


        KNEE_K_LSWING,
        KNEE_B_LSWING,
        KNEE_THETA_LSWING,
        ANKLE_K_LSWING,
        ANKLE_B_LSWING,
        ANKLE_THETA_LSWING

        LOAD_ESTANCE,
        KNEE_THETA_LSWING_TO_ESTANCE,


    ):
        data = dynamic_message_pb2.Data(
            values={
                "Knee k EStance": KNEE_K_ESTANCE,
                "Knee b EStance": KNEE_B_ESTANCE,
                "Knee theta EStance": KNEE_THETA_ESTANCE,
                "Ankle k EStance": ANKLE_K_ESTANCE,
                "Ankle b EStance": ANKLE_B_ESTANCE,
                "Ankle theta EStance": ANKLE_THETA_ESTANCE,

                "Load LStance": LOAD_LSTANCE,
                "Ankle theta EStance To LStance": ANKLE_THETA_ESTANCE_TO_LSTANCE,

                "Knee k LStance": KNEE_K_LSTANCE,
                "Knee b LStance": KNEE_B_LSTANCE,
                "Knee theta LStance": KNEE_THETA_LSTANCE,
                "Ankle k LStance": ANKLE_K_LSTANCE,
                "Ankle b LStance": ANKLE_B_LSTANCE,
                "Ankle theta LStance": ANKLE_THETA_LSTANCE,

                "Load ESwing": LOAD_ESWING,


                "Knee k ESwing": KNEE_K_ESWING,
                "Knee b ESwing": KNEE_B_ESWING,
                "Knee theta ESwing": KNEE_THETA_ESWING,
                "Ankle k ESwing": ANKLE_K_ESWING,
                "Ankle b ESwing": ANKLE_B_ESWING,
                "Ankle theta ESwing": ANKLE_THETA_ESWING,

                "Knee Theta ESwing To LSwing": KNEE_THETA_ESWING_TO_LSWING,
                "Knee DTheta ESwing To LSwing": KNEE_DTHETA_ESWING_TO_LSWING,


                "Knee k LSwing": KNEE_K_LSWING,
                "Knee b LSwing": KNEE_B_LSWING,
                "Knee theta LSwing": KNEE_THETA_LSWING,
                "Ankle k LSwing": ANKLE_K_LSWING,
                "Ankle b LSwing": ANKLE_B_LSWING,
                "Ankle theta LSwing": ANKLE_THETA_LSWING,

                "Load EStance": LOAD_ESTANCE
                "Knee Theta LSwing To EStance": KNEE_THETA_LSWING_TO_ESTANCE
            }
        )
        response = stub.SendData(data)
        print("Data sent:", data.values)

    # Receive Data
    def receive_data():
        response = stub.ReceiveData(dynamic_message_pb2.Empty())
        print("Data received:", response.values)

    try:
        t = 0
        while True:
            send_data(1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1)
            # receive_data()
            time.sleep(0.01)
            t += 0.01
    except KeyboardInterrupt:
        print("Client stopped.")


if __name__ == "__main__":
    run_client()
