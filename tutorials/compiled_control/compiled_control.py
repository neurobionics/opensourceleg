import os

from opensourceleg.control.compiled_controller import CompiledController

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
my_linalg.define_inputs(
    [("vector1", my_linalg.types.Vector3D), ("vector2", my_linalg.types.Vector3D)]
)
my_linalg.define_outputs([("result", my_linalg.types.c_double)])

vector1 = my_linalg.types.Vector3D()
vector2 = my_linalg.types.Vector3D()

vector1.x = 2.5
vector1.y = 5.3
vector1.z = 7.3
vector2.x = 4.2
vector2.y = 3.2
vector2.z = 8.0

my_linalg.inputs.vector1 = vector1  # type: ignore
my_linalg.inputs.vector2 = vector2  # type: ignore

outputs = my_linalg.run()

print(f"Dot product: {outputs.result}")
