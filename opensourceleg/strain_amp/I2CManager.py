from smbus2 import SMBus
import time
import numpy as np
import os

class I2CManager:
    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    """
    Used to keep track of one instantation of the class to make a singleton object
    """
    
    def __new__(cls, bus):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(I2CManager, cls).__new__(cls)
            print("Initializing I2C Manager")
            cls._instance.bus = SMBus(bus)
            # create a python-can notifier object, which motors can later subscribe to
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self, bus):
        """
        ALl initialization happens in __new__
        """
        pass
        
    def __del__(self):
        """
        # shut down the bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        self.bus.close()