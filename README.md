# LPCXpresso-and-servo-control
## Import instructions
Using MCUXpresso IDE, right click in the Project Explorer tab and choose the 'Import' option. Select 'Existing Projects into Workspace' and browse to the .zip archive.
## Mounting
Connect the X-NUCLEO MEMS IKSA01A2 on top of the LPCXpresso Board.

* Mount a servomotor underneath the board (make sure that the rotor is in its neutral position). For our experiment the servomotor used is a Parallax 900-00005.
* Power source for the servo can be obtained connecting it through the MEMS.
* The PWM signal is taken from the **P0_12** pin.

![Montage](https://i.imgur.com/k3gS6go.png)
## Notes
If you are interested in observing the sensor measurements, open **main.c** and uncomment the lines:
```
Display_Readings(gyro,acc,&yaw,&roll,&pitch_f);        
```
Using them will greatly hamper the performance of the self correction. These debug messages can be read using the integrated serial terminal or using applications like [PuTTY](https://github.com/larryli/PuTTY).
## Authors
@strawmark

@elibighouse
