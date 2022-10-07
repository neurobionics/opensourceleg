import strain_methods as sm
import numpy as np
import unittest

# format
# ezI2Cbuf[0]: 0000 0000
# ezI2Cbuf[1]: 0000 1111
# ezI2Cbuf[2]: 1111 1111
# ezI2Cbuf[3]: 2222 2222
# ezI2Cbuf[4]: 2222 3333
# ezI2Cbuf[5]: 3333 3333
# ezI2Cbuf[6]: 4444 4444
# ezI2Cbuf[7]: 4444 5555
# ezI2Cbuf[8]: 5555 5555


class TestStrainMethods(unittest.TestCase):

    def test1(self):
        data = [0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF]
        test_expected = np.array([0x0000, 0xFFFF, 0x0000, 0xFFFF,0x0000, 0xFFFF])
        test = sm.unpack_uncompressed_strain(data)
        with np.printoptions(formatter={'int':hex}):
            print(test)
        self.assertEqual(test.all(),test_expected.all() )

    def test2(self):
        print("\n")
        data = [0x00, 0x0F, 0xFF, 0x00, 0x0F, 0xFF, 0x00, 0x0F, 0xFF]
        test_expected = np.array([0x000, 0xFFF, 0x000, 0xFFF,0x000, 0xFFF])
        test = sm.unpack_compressed_strain(data)
        with np.printoptions(formatter={'int':hex}):
            print(test)
        self.assertEqual(test.all(),test_expected.all() )

    def test3(self):
        print("\n")
        data = [ 0x00, 0x01, 0x11, 0x22, 0x23, 0x33, 0x44, 0x45, 0x55 ]
        test_expected = np.array([0x000, 0x111, 0x222, 0x333, 0x444, 0x555])
        test = sm.unpack_compressed_strain(data)
        with np.printoptions(formatter={'int':hex}):
            print(test)
        self.assertEqual(test.all(), test_expected.all())

    def test4(self):
        print("\n")
        test_cases = [2048,0,4095,2000,2100]
        for val in test_cases:
            input = np.array([val,val,val,val,val,val])
            loadcell1 = sm.strain_V_to_wrench(input)
            strain1 = sm.wrench_to_strain_V(loadcell1)
            print("Input: {}".format(input))
            print("Loadcell strain: {}".format(loadcell1))
            print("Retrieve Input: {}".format(strain1))
            print()


if __name__ == "__main__":
    
    unittest.main(verbosity=2)

    
    


