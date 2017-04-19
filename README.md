# MelbusBluetoothInterface
Volvo melbus with Bluetooth integration

This repository keeps track of my efforts to integrate bluetooth within my car stereo (HU850)

Credits are for those who analysed the low-level protocols involved, an whose code i've borrowed, see comments within the code with links to original authors


## Requirements
* Bluetooth module (OVC3860)
* Arduino pro mini
* Solderings skills


## Background
I've connected my setup via the RTI 13-DIN plug, but there really isn't any difference.


## Contents
* *hw* Contains hardware schema's
* *sw* Contains (cough) production code
* *testing* various efforts to crack the displaying of text\*

## Current state
* Playback of bluetooth audio works very nicely
* The control via headunit or steering-wheel feels 'native'


*\* unforunatly everybody who seems to have cracked the sending of text to the head-units tries to commercialize this knowledge and doenst feel like sharing with the community... so should my attempts to get something on the display be successfull, i'm sure to share these results, for anybody breave enough to assemble their own hardware. You can always revert to an commercial version, free of bugs, and fieldtested. Should anybody have knowlage as to how send text, do let me know, or contribute!*
