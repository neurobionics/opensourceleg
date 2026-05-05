import os

from opensourceleg.control.compiled import CompiledController

my_linalg = CompiledController(
    library_name="lin_alg.so",
    library_path=os.path.dirname(__file__),
    main_function_name="dot_product_3d",
    initialization_function_name=None,
    cleanup_function_name=None,
)

my_linalg.define_type(
    "Vector3D",
    [
        ("x", my_linalg.types.c_double),
        ("y", my_linalg.types.c_double),
        ("z", my_linalg.types.c_double),
    ],
)
my_linalg.define_inputs(input_list=[("vector1", my_linalg.types.Vector3D), ("vector2", my_linalg.types.Vector3D)])
my_linalg.define_outputs(output_list=[("result", my_linalg.types.c_double)])

vector1 = my_linalg.types.Vector3D()
vector2 = my_linalg.types.Vector3D()

vector1.x = 0.6651
vector1.y = 0.7395
vector1.z = 0.1037
vector2.x = -0.7395
vector2.y = 0.6716
vector2.z = -0.0460

my_linalg.inputs.vector1 = vector1
my_linalg.inputs.vector2 = vector2

outputs = my_linalg.run()

print(f"Dot product: {outputs.result}")
