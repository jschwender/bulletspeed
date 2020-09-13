# bulletspeed
Measunring the speed of bullets with laser barrier

A rail is fixed to the gun bore. It consists of a laser that is mounted perpendicular to the trajectory path. the laser beam is deflected two times with mirrors by 90° in order to intersect with the trajectory two times in a fixed, known distance. A PIN diode detects the laser. A bullet traversing the trajectory path interrupts the laser two times in a time distance. This time distance gives a signal comprising of two pulses. This signal is fed into interrupt input if a arduino ini order to measure the time between the two pulse edges.

The method has a good resiliance against muzzle smoke, as the laser is quite powerful and detection between darkening by smoke and the bullet edge is quite good. Even with that simple arduino measurement set up, the precision is good even with high speeds. The setup is usable under any light conditions, the only limitation  in direct sun light is a reduced readybility of the display. The light detection is not that simple, as very short rise times are required. This is archieved by utilizing a fast comparator in combination with a PIN photo diode.

Note: Code variables and comments are in German language
