### Analog Module
This is a part of the implementation for the Ultracold Control System provided by ParkerLab at Georgia Tech.
An Analog Module can generate a sequence of voltage signals over 4 channels. The voltage signals are generated by the AD5686 DAC chip and amplified by the AD5750-2 amplifier chip. The gain of the AD5750-2 can determine the voltage range mode. The mode can be set separately for each channel.
### Hardware
The corresponding hardware design can be find in https://github.com/parkerlab-external/UltracoldPCB-DAC.
### Protocol
To update the sequence the module accepts command lists in the following format.
```
 × l    #bytes      1         1            1
        value       'G'       slot id      range mode 
 × m    #bytes      1         1            1
        value       'E'       slot id      chkpt count 
 × n    #bytes      1         1            2            4               4
        value       'U'       slot id      chkpt pos    duration        value 
 × 1    #bytes      1
        value       'T'
```
The four slots in the MCU do indeed correspond to the four output channels.
The `duration` and `value` are polymorphic: a negative `duration` represents a jump to the corresponding absolute `value`, while a positive duration means that the `value` is an increment that's executed per cycle, for a total of `duration` cycles.
An implementation of the backend can be found here: https://github.com/parkerlab-external/UltracoldSequenceRunner.
### Configurables
The device specific configuration of the UART address can be set in `config_device.h`.