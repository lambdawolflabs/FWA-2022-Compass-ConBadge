# FWA-2022-Compass-ConBadge
FWA 2022 Compass ConBadge from the Intro To DIY Electronics &amp; Electrical Engineering Panel!


Slides: https://docs.google.com/presentation/d/1c4_kWb6G9Z7fMHBQ-3chpQz8QXO9tOMy/edit?usp=sharing&ouid=112541527262173468485&rtpof=true&sd=true

Recorded Panel: https://www.youtube.com/watch?v=K9LNleyEv4E 

Purpose: Semi-basic, semi-intelligent, semi-interactive conbadge with LED compass function, two BJT transistor multivibrator flasher (can be disabled by MCU), MCU runs the show and built in NFC tag.

Specs:
	
	>Operating/Storage Ambient Temperature Range: 0°C to 40°C.
	
	>Operating/Storage Ambient Humidity Range: 10% to 95% RH.
	
	>Atlantis TLE Theme. Atlantean Crystal Necklace in color (Blue soldermask) and overall shape. 
		
		>PCB is in a truncated cone/prism pill shape with a multi-angled (rounded) arrow end at the large side of the truncation. ~2-3" long by ~1.5" wide.
			
			>Large arrow rounded end is for the compass sensor and compass LEDs.
			
			>Shaft holds most of the electronics (MCU, NFC transiever, pushbutton, etc).
			
			>Smaller flat end holds the battery mount, NFC antenna and lanyard hole.
			
			>NFC antenna is circular with a similar (copyright legal) look to the Eye of Atlantis.


Goals & Requirements:
	
	-EU ROHS3 COMPLIANT
	
	-NO EXPLICIT ESD PROTECTION.
	
	-Has a hole for a keychain or lanyard hook.
	
	-Powers off a single CR2032.
		
		-Lasts atleast 12 hours of continuous compass function (MCU/Compass active + compass LED) and nominal frequency multivibrator flasher (one LED lit at a time): <7mA
		
		-Lasts atleast 24 hours of continuous nominal frequency multivibrator flasher alone (one LEDs lit at any given time)
		
		-Lasts atleast 8 hours of continuous EVERYTHING ON/MAXED OUT operation.
	
	-Simple 2 BJT transistor astable multivibrator circuit.
	
	-Simple integrated NFC rewriteable tag that can be programmed via smartphone.
		
		-Antenna is fully made in PCB FR-4.
		
		-NFC tag harvest output can be used to power downstream circuits without coin cell (optional).
	
	-Compass function:
		
		-Three axis magnetoer/compass sensor (+ accel and/or gyro? optional) for determining which way is north.
		
		-MCU reads this off.
		
		-Charlieplexed LED array in 8 point circle. Only a single LED lights up to point which way is north.
		
		-Reprogrammable MCU (Only wakes up upon button hold to update compass).
	
	-Optional White LED for general illumination.
	
	-Optional RGB LED for customization (Vanilla FW reads the RGB value stored in the NFC chip).
	
	-Additional pins from the MCU broken out for UPDI reprogramming (FTDI pinout), two additional GPIOs (+ VDD & GND).
	
	-OSHPark Prototype PCB Process Compatible 

