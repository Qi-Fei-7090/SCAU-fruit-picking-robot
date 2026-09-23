float32 voltage   # no use
float32 current
int32 rpm
int32 tick
float32 temperature

# bit0: reserve
# bit1: encoder ABZ/disconnect
# bit2: encoder UVW/inner encoder
# bit3: encoder count/encoder CRC
# bit4: driver temperature error
# bit5: Over voltage
# bit6: Lower voltage
# bit7: Over current
# bit8: Absorption resistance
# bit9: Position following error
# bit10: Logic low voltage
# bit11: motor/driver iit
# bit12: tick freq is too high
# bit13: motor temperature error
# bit14: Motor excitation alarm/encoder communication
# bit15: EEPROM
int16   fault