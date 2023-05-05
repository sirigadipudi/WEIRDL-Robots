# Dynamixel Reset

This is a guide to set up a reset mechanism using dynamixel motors (or in general, a guide on how to use dynamixel motors with python).



#### How to use run the code

In order to reset the mechanism you will have to run:

```
from Dynamixel_reset.motor import Motor
motor = Motor()
motor.reset()
```



#### Changing config info

It is possible that the code doesn't work at first or the motor doesn't move in the desired range.

If the motor is not being detected (you don't see the message: "Dynamixel has been successfully connected" when initializing the Motor class). It is probably because either you are using a different motor and will have to change some parameters, or the motor ID, the protocol version or the device name are different from the given ones. In either case you will have to change some of the parameters of the init. (TODO: move all that to a config file).

1. If the motor you are using is different, go to [Documentation](https://emanual.robotis.com/docs/en/dxl/) select your motor and look for the correct addresses for your motor at section 2 (control Table).
2. In order to get the protocol version, motor ID and the device name, use [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and scan your motor to find those parameters.
3. Finally, in order to change the range to the desired one, change the ```self.HEIGHT_RANGE ``` parameter. 



