/** dot_product_3d.cpp
An example cpp file to demonstrate the compiled controller functionality
in the opensourceleg library.
Kevin Best
University of Michigan
October 2023
**/

struct Vector3D{
    double x, y, z;
};

struct Inputs{
    Vector3D vector1, vector2;
};

struct Outputs{
    double result;
};

extern "C" void dot_product_3d(Inputs* inputs, Outputs* Outputs) {
  Outputs->result = inputs->vector1.x * inputs->vector2.x
                    + inputs->vector1.y * inputs->vector2.y
                    + inputs->vector1.z * inputs->vector2.z;
}
