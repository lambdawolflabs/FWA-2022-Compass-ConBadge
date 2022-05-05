/*
 * FWA_2022_Compass_ConBadge_Arduino_FW.ino
 *
 * Created: 4-15-2022 thru 5-4-2022
 * Author : @LorneChrones / Nick / Lambda Wolf Labs
 * Repo: https://github.com/lambdawolflabs
 * Project/FW Number: FWA 2022 Compass ConBadge
 * Revision: Release
 *
 * MCU Used: ATTINY814
 * MCU Datasheet: http://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en599388
 * MCU Errata: http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny214-414-814-Errata-and-Clarification-DS40002115C.pdf
 *
 * MCU Notes: Might recommend upgrading to a ATTINY1614 in the future for more FLASH space (but chip shortage *pain* ;_;)
 *
 * Programming: https://github.com/microchip-pic-avr-tools/pymcuprog
 * (Or Arduino IDE built in //SerialUPDI or ATMEL-ICE tool)
 *
 * Arduino Core Used (Installation instructions provided too) here: https://github.com/SpenceKonde/megaTinyCore
 * megaTinyCore Arduino Recommended Settings To Use For the FWA 2022 Compass ConBadge (Bootloader burn required for some):
 *    Board:                            ATTINY814         (No optiboot)
 *    Chip:                             ATTINY814
 *    Clock:                            4MHz Internal           
 *    millis()/micros() Timer:          Enabled (Default Timer)
 *    Startup Time:                     8ms
 *    BOD Voltage Level:                2.1V  (Unofficial)
 *    BOD Mode when Active/Sleeping:    Enabled/Enabled
 *    Save EEPROM:                      EEPROM Retained
 *    attachInterrupt() Version:        On all pins, like usual
 *    printf():                         Default (doesn't print floats)
 *    Wire Library Mode:                Master or Slave
 *    
 * Additional Fuse Notes:
 *    Default APPEND, Default BOOTEND, CRC disabled, RESET pin function is UPDI. Disable Timer D pin output.
 *    Due to a silicon bug, NEVER SET OSCCFG.OSCLOCK TO '1' (Errata 2.2.2).
 *    Leave device unlocked, fuse & lock bits can be editted.
 * 
 * Compass Notes: I've provided these compasses as largely uncalibrated so they aren't guaranteed to point North by default.
 *                However I've "rough calibrated" them somewhat and left in additional links to how you can properly calibrate them yourself.
 *    
 *    
 * Boot Up:
 *    1.LED Multivibrator blinkers startup up enabled at power on. (D1 and D2)
 *    2.MCU (U3) cycles the White LED (D3) and then the R, G and then B LEDs (in the RGB package, D12). 
 *    3.MCU turns off the RGB LED.
 *    4.MCU I2C scans for the ST25DV NFC chip (U1), if it detects it, it turns briefly sets the RGB LED to Green, if not it briefly sets the RGB LED to Red. 
 *        4a.If the MCU found the NFC chip then it'll read its contents and load any user configuration settings into memory for later use. (See "NFC User Configuration Commands" section for command set, syntax and usage)
 *    5.MCU I2C scans for the LIS3MDL Compass chip (U2), if it detects it, it turns briefly sets the RGB LED to Blue, if not it briefly sets the RGB LED to Red. 
 *    6.MCU sequence turns on the compass ring LEDs (D10, D5, D6, D9, D11, D4, D8 and D7 in that order) clockwise. 
 *    7.MCU turns of all of the compass ring LEDs, the RGB LED and white LED and enters Mode 0.
 *
 * Modes:
 *    Swap between modes by holding the pushbutton for longer than one second. Short button presses (<1 second)toggles/runs the Action for that particular mode.
 *    When a new mode is entered, a corresponding LED on the LED compass ring is blinked 3 times. Starting clockwise at the top LED (D10 is Mode 0, D5 is Mode 1, D6 is Mode 2, etc).
 *    
 *    
 *    Mode 0: Multivibrator enabled, everything else off.    - Short Press: Toggles Multivibrator On/Off.
 *    Mode 1: RGB LED: Solid Blue, everything else off.    - Short Press: Toggles RGB LED from Solid Blue to Solid Green to Solid Red  then back to Solid Blue.
 *    Mode 2: White LED: Solid White, everything else off.    - Short Press: Toggles White LED from Solid White to 1Hz Blink to SOS Morse Pattern then back to Solid White.
 *    Mode 3: Green LED Ring Demo: Binary Count, Fast Spinner (CW/CCW), Persistence Of Vision Full Ring. - Short Press: Cycles between Green LED Ring Fast Spinner clockwise, Green LED Fast Spinner counterclockwise, Persistence Of Vision Solid Green Ring & Persistence Of Vision Binary Count.
 *    Mode 4: Compass Demo/Green LED ring points north according to U2. - Short Press: N/A
 *    Mode 5: Custom Profile Defined By NFC EEPROM (User can enable whatever LEDs or LED patterns to whatever values simultaneously in this mode, see "NFC User Configuration Commands" section for supported commands/settings).   - Short Press: Reread/rescan NFC EEPROM memory for user configuration.
 *    Mode 6: Rainbow Random Mode, randomly jumps around between different LEDs being enabled/disabled at a 3 sec interval.   - Short Press: N/A
 *    Mode 7: Everything Off (Including Multivibrator), MCU goes into sleep mode.             - Short Press: Wake up and go to next mode
 *    
 *     
 * NFC User Configuration Commands:
 *    Use only "text mode" NFC entries when writing to the NFC chip to program it. Its safe to still mix in other entries like socials, URLs, etc in the NFC.
 *	  Text Mode Entry Programming Format (without quotations): "led=X!"
 *    Where X can be "0" thru "7", "r" or "R", "g" or "G", "b" or "B", "w" or "W", "p" or "P" and "n" or "N"
 *    "0" thru "7" simply solid light up one of the LEDs on the Green LED ring (clockwise)
 *    "r" or "R", "g" or "G", "b" or "B" simply solidly light up either the Red, Green or Blue LED in the RGB LED chip.
 *    "w" or "W" simply solidly lights up the White LED.
 *    "p" or "P" does a progress bar animation with the green LED ring.
 *    "n" or "N" does a Night Rider/KITT (or Cylon) eye animation with the green LED ring.
 *   
 *
 * Lorne: Feel free to use this code and modify, redistribute, etc with it. I'm not a coder by training (EE by trade), so there's plenty of improvments here.
 *
 * Lorne: Thank you for attending the panel and I hope you enjoyed it / learned something new too! ^_^
 * 
 * GPL v3 License
 *
 * 1. Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * 2. The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * 3. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * 4. This software is strictly NOT intended for safety-critical or life-critical applications.
 * If the user intends to use this software or parts thereof for such purpose additional
 * certification is required.
 *
 */ 

//Includes & Defines
#include <Wire.h>
#include <Math.h>
#include <avr/sleep.h>


//Math predefines
#define twoPI    6.283185307
#define rad2deg   57.29577951
#define rad90deg  1.570796327
#define rad270deg 4.71238898



//Pin Definitions
#define EXP_A_TXD_A1  8   //PIN1_bm //PA1, Output
#define EXP_B_RXD_A2  9   //PIN2_bm //PA2, Input
#define INT_MUX       10  //PIN3_bm //PA3, Input/ADC
#define LED_CHARLIE_A 0   //PIN4_bm //PA4, Output
#define LED_CHARLIE_B 1   //PIN5_bm //PA5, Output
#define LED_CHARLIE_C 2   //PIN6_bm //PA6, Output
#define LED_CHARLIE_D 3   //PIN7_bm //PA7, Output

#define I2C_SCL       7   //PIN0_bm //PB0, Output/I2C SCL
#define I2C_SDA       6   //PIN1_bm //PB1, Bidir/I2C SDA
#define PB_WAKE       5   //PIN2_bm //PB2, Input
#define BLINK_DIS     4   //PIN3_bm //PB3, Output


//Additional LED defines for later use
#define White    8
#define RGBBlue   9
#define RGBGreen  10
#define RGBRed    11


//NFC User Defined Programs
#define ProgressBar 12
#define NightRider	13
#define Idle		255


//I2C Device Definitions
#define ST25DV_eeprom_addr    0x53
#define ST25DV_registers_addr 0x57
#define LIS3MDL_addr          0x1C


//LIS3MDL Register names & addresses
#define LIS3MDL_WHO_AM_I  	0x0F
#define LIS3MDL_CTRL_REG1   0x20
#define LIS3MDL_CTRL_REG2   0x21
#define LIS3MDL_CTRL_REG3   0x22
#define LIS3MDL_CTRL_REG4   0x23
#define LIS3MDL_CTRL_REG5   0x24
#define LIS3MDL_STATUS_REG  0x27
#define LIS3MDL_OUT_X_L     0x28
#define LIS3MDL_OUT_X_H     0x29
#define LIS3MDL_OUT_Y_L     0x2A
#define LIS3MDL_OUT_Y_H     0x2B
#define LIS3MDL_OUT_Z_L     0x2C
#define LIS3MDL_OUT_Z_H     0x2D
#define LIS3MDL_TEMP_OUT_L  0x2E
#define LIS3MDL_TEMP_OUT_H  0x2F
#define LIS3MDL_INT_CFG     0x30
#define LIS3MDL_INT_SRC     0x31
#define LIS3MDL_INT_THS_L   0x32
#define LIS3MDL_INT_THS_H   0x33



//Globals
//(Relatively) Simple compass calibration:
//https://www.bot-thoughts.com/2011/04/quick-and-dirty-compass-calibration-in.html
int16_t mag_x = 0;
  int16_t mag_x_offset = -1020;   //You'll need to rerun manual calibration to find the offsets (which are the averages of the min and max values collected);
int16_t mag_y = 0;
  int16_t mag_y_offset = -525;
int16_t mag_z = 0;
  int16_t mag_z_offset = 1137;
uint8_t mag_buff = 0;

double mag_x_double = 0.0;
  double mag_x_scale = -7.608;
double mag_y_double = 0.0;
  double mag_y_scale = 7.097;
//double mag_z_double = 0.0;
  //double mag_z_scale = 1;
double angle_double = 0.0;

int8_t angle = 0;

//Advanced compass calibration:
//https://github.com/YuriMat/MagMaster
//float calibrated_values[3];  //Xcal, Ycal, Zcal
//float uncal_values[3];       //""
//float mag_x_cal;
//float mag_y_cal;
//float mag_z_cal;

char buff = 0;

char button_held = 0;



//Sytem Wide Globals & Mode state variables.
volatile char current_mode = 255;             //For debugging purposes, do not set the current_mode to match next_mode on startup. Set next_mode here to zero to enter mode zero on boot.
volatile char next_mode = 0;
volatile char userNFCProgrammedAction = 0;
volatile char shortPressDetected = false;

volatile unsigned long millisTimestamp = 0;

volatile unsigned long millisBinaryCount = 0;
volatile unsigned long millisModeFive = 0;

int randSeedFromNFC = 1111;

volatile char modeOneStorage = 0;
volatile char modeTwoStorage = 0;
	volatile char modeTwoStepCounter = 0;
	volatile char modeTwoSOSCounter = 0;		
volatile char modeThreeStorage = 0;
	volatile char modeThreeSpinnerPos = 0;
	volatile char modeThreeBinaryCount = 0;
	volatile char modeThreePOVSpeed = 75;
volatile char modeFiveStorage = 0;
	volatile char modeFiveProgressBar = 0;
	volatile char modeFiveNightRider = 0;
	


//Charlieplex LED Definitions, in order of clockwise direction green LEDs, then the white LED and lastly the RGB LEDs.
//  DCBA
//0bxxxx
//                  D to A,C to B , C to D,D to C , A to D,B to C , B to A,A to B , A to C,C to A , B to D,D to B
//                  D10   ,D5   , D6    ,D9     , D11   ,D4     , D8    ,D7     , D3    ,D12B   , D12G  ,D12R
const unsigned char charlieDIR[] = {0b1001,0b0110 , 0b1100,0b1100 , 0b1001,0b0110 , 0b0011,0b0011 , 0b0101,0b0101 , 0b1010,0b1010};
const unsigned char charlieOUT[] = {0b1000,0b0100 , 0b0100,0b1000 , 0b0001,0b0010 , 0b0010,0b0001 , 0b0001,0b0100 , 0b0010,0b1000};





//Func Defines

//Charlieplexe routine to drive the LEDs output
void charlieplex(unsigned char LEDNumber, unsigned char Enabled) {
  if (Enabled == 0) {
    VPORTA.OUT &= 0x0F;
    _NOP();
    _NOP();
    VPORTA.DIR &= 0x0F;
    _NOP();
    _NOP();
  } else {
    VPORTA.OUT &= 0x0F;
    _NOP();
    _NOP();
    VPORTA.DIR &= 0x0F;
    _NOP();
    _NOP();
    
    VPORTA.DIR |= ((charlieDIR[LEDNumber]) << 4);
    _NOP();
    _NOP();
    VPORTA.OUT |= ((charlieOUT[LEDNumber]) << 4);
    _NOP();
    _NOP();
  }
}



//Boot Up Routine
void BootUp() {

  //Setup CPU clocks.
  CCP = 0xD8;
  CLKCTRL.MCLKCTRLA = 0x00; //Run CLK_CPU at 4MHz from the internal 16MHz RC Osc, factory temp/accuracy compensation values automatically loaded
  CCP = 0xD8;
  CLKCTRL.MCLKCTRLB = 0x03;
  
  
  //CLKCTRL.MCLKLOCK = 0x00;  //Leave the MCLKCTRLA, MCLKCTRLB, OSC20MCALIBA and OSC20MCALIBB registers available to modify during runtime.
  CCP = 0xD8;
  CLKCTRL.OSC20MCTRLA = 0x02; //Force ably start the 16MHz RC Oscillator in all modes even when not requested (incl. standby)
  CCP = 0xD8;
  CLKCTRL.OSC32KCTRLA = 0x02; //Force ably start the 32kHz ULP Oscillator in all modes even when not requested (incl. standby)
  CCP = 0xD8;
  CLKCTRL.XOSC32KCTRLA = 0x00;//Disable the 32.768kHz Ext XTAL circuitry entirely.
  
  while((CLKCTRL.MCLKSTATUS & 0x11) == 0x01) {  //Wait until OSC20MS is stable and SOSC/CLK_MAIN is no longer clock switching.
    _NOP();
  }

  
  //Setup BOD & VLM, most of this is setup in fuses and can be ignored.
  CCP = 0xD8;
  BOD.CTRLA = 0x15;     //125Hz sample, BOD ENABLED @ 125Hz in both Active & Sleep
  BOD.CTRLB = 0x01;     //BODLEVEL1 2.15V, W.C. brownout at 2.032V which is still above the 1.9V minimum board wide op voltage.
  BOD.VLMCTRLA = 0x00;    //VLM Thres 5% above BOD, thus 2.25V VLM trigger, this is still above the EOL voltage of CR2032s at 2V.
  BOD.INTCTRL = 0x00;     //Disable interrupts for now, will comeback later after setup...
   
  
  //Setup GPIO
  charlieplex(0,0);         //Disable Charlieplex
  PORTA.PIN4CTRL = 0x04;    //Disable input buffers and disable pullups on all the charlieplex pins since they'll only be used for outputs.
  PORTA.PIN5CTRL = 0x04;
  PORTA.PIN6CTRL = 0x04;
  PORTA.PIN7CTRL = 0x04;
  
  PORTA.PIN3CTRL = 0x04;    //Temporarily disable the input buffer to INT_MUX. It'll become enabled along with interrupts when the I2C devices are setup.


  //Set BLINK_DIS as an output pin and ensure it starts LOW to enable the LED BJT multivibrator.
  pinMode(BLINK_DIS, OUTPUT);
  digitalWrite(BLINK_DIS, LOW);


  
  //Set the was-alternate, now-primary UART output to an OUTPUT pin prior to //Serial.begin
  //pinMode(EXP_A_TXD_A1, OUTPUT);


  //Serial.begin(9600);

  //Setup PORTMUX for the alternate pins on the UART for the expansion pins, TWI/I2C pins on standard pinout, EVOUT not used, SPI not used.
  PORTMUX.CTRLA = 0x00;
  PORTMUX.CTRLB = 0x01;   //Specifically for UART alt pins on PA1/PA2
  PORTMUX.CTRLC = 0x00;

  //Serial.println(" ");


  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.print("FWA 2022 Compass Badge startup. ");
  //Serial.println("All clocks, pins, BOD previously setup.");
  

  delay(1000);

  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.println("RGB + White exercise");
  charlieplex(White,1);
  delay(500);
  charlieplex(RGBRed,1);
  delay(500);
  charlieplex(RGBGreen,1); 
  delay(500);
  charlieplex(RGBBlue,1);
  delay(500);

  charlieplex(0,0);
  delay(500);


  Wire.begin();

  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.print("Scanning for ST25DV NFC chip...");
  //Check to see if the ST25DV is there by pinging out and reading the UID byte 6 register (fixed to 0x02)
  //If detected then flash the RGB to green, otherwise flash red.
  Wire.beginTransmission(ST25DV_registers_addr); 
  Wire.write(0x00);
  Wire.write(0x1E);
  Wire.endTransmission(false);

  Wire.requestFrom(ST25DV_registers_addr, 1);
  while (Wire.available()) {
    buff = Wire.read();   
  }

  if(buff == 0x02) {
    charlieplex(RGBGreen,1);
    //Serial.println("FOUND!");

	LoadUserNFCConfigurations();
  } else {
    charlieplex(RGBRed,1);
    //Serial.println("NOT FOUND!");    
  }

  delay(500);
  
  charlieplex(0,0);
  
  delay(500);

  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.print("Scanning for LIS3MDL Compass chip...");
  //Check to see if the LIS3MDL is there by reading the WHO_AM_I register (fixed to 0x3D);
  //If detected then flash the RGB to blue, otherwise flash red.
  Wire.beginTransmission(LIS3MDL_addr);   //Access the ST25DVxx User Memory, write
  Wire.write(LIS3MDL_WHO_AM_I);
  Wire.endTransmission(true);

  Wire.requestFrom(LIS3MDL_addr, 1);
  while (Wire.available()) {
    buff = Wire.read();   
  }

  if(buff == 0x3D) {
    charlieplex(RGBBlue,1);
    //Serial.println("FOUND!");
  } else {
    charlieplex(RGBRed,1);
    //Serial.println("NOT FOUND!");
  }
  
  delay(500);
  
  charlieplex(0,0);
  
  delay(500);

  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.println("LED Compass Ring Exercise");
  for(char x = 0 ; x < 8 ; x++) {
    charlieplex(x,1);
  
    delay(100);
  }

  charlieplex(0,0);

  //Enable PB_WAKE internal pull-up. 50kOhm worst case internall pull-up and 100nF 5xRC debounce circuit ->   25mSec to wait for a full charge minimum
  pinMode(PB_WAKE, INPUT_PULLUP);
  delay(25);

  digitalWrite(BLINK_DIS, LOW);
}



//Setup the LIS3MDL compass sensor for later use
void LIS3MDL_Setup() {
  //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
  //Serial.println("Setting up LIS3MDL compass sensor settings.");
  //LIS3MDL temp sensor enabled, medium performance on X/Y axes, 10Hz ODR, no FAST_ODR, self-test disabled
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_CTRL_REG1);
  Wire.write(0xB0);       
  Wire.endTransmission(true);

  //LIS3MDL 16 gauss full scale
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_CTRL_REG2);
  Wire.write(0x60);       
  Wire.endTransmission(true);

  //LIS3MDL low power mode disabled, single conversion mode
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_CTRL_REG3);
  Wire.write(0x01);       
  Wire.endTransmission(true);
  
  //LIS3MDL medium performance mode on Z axis
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_CTRL_REG4);
  Wire.write(0x04);       
  Wire.endTransmission(true);
  

  //LIS3MDL no FAST_READ, do not block data updates
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_CTRL_REG5);
  Wire.write(0x00);       
  Wire.endTransmission(true);
  

  //LIS3MDL disable interrupts
  Wire.beginTransmission(0x1C);
  Wire.write(LIS3MDL_INT_CFG);
  Wire.write(0x08);       
  Wire.endTransmission(true);
}


//Read the ST25DV NFC EEPROM contents to look for any user custom configuration modes, if found then set the userDefinedMode variable.
void LoadUserNFCConfigurations() {
	//Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
	//Serial.println("Reading ST25 NFC EEPROM Contents, scanning for user programmed codes.");
	char st25_nfc_eeprom_byte = ' ';

	char keyword_found = false;
	int keyword_matching_chars = 0;
	char keyword_index = 0;
	char keyword[] = "led=";
	char end_keyword = '!';

	char keyword_value = 'r';

	for (uint16_t st25_nfc_eeprom_addr = 0x0000 ; st25_nfc_eeprom_addr <= 0x01FF ; st25_nfc_eeprom_addr++) {
		Wire.beginTransmission(ST25DV_eeprom_addr);   //Access the ST25DVxx User Memory, write
		Wire.write( (st25_nfc_eeprom_addr & 0xFF00) >> 8 );
		Wire.write( (st25_nfc_eeprom_addr & 0xFF) );
		Wire.endTransmission(true);

		Wire.requestFrom(ST25DV_eeprom_addr, 1);
		while (Wire.available()) {
		  st25_nfc_eeprom_byte = Wire.read();   
		}

		randSeedFromNFC ^= st25_nfc_eeprom_byte; 
    
		if (st25_nfc_eeprom_byte == keyword[keyword_index]) {
		  keyword_matching_chars++;
		  keyword_index++;
		} else {
		  keyword_matching_chars = 0;
		  keyword_index = 0;
		}

		if((keyword_matching_chars == strlen(keyword)) && (keyword_found == false)) {
		  keyword_found = true;
	    //Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
		  //Serial.println("NFC User Programmed Entry found!");
		}

		if((keyword_found)) {
			if(st25_nfc_eeprom_byte == end_keyword) {
				break;
			} else {
				keyword_value = st25_nfc_eeprom_byte;
			}
		}
		
		
		/*
		//Serial.print("Addr: ");
		//Serial.print(st25_nfc_eeprom_addr, HEX);
		//Serial.print(" contents: ");
		//Serial.print(st25_nfc_eeprom_byte);
		//Serial.println(" ");
		delay(1);
		*/
	}

	switch(keyword_value) {
		//Solid Green
		case 'g':
		case 'G':
			userNFCProgrammedAction = RGBGreen;
			break;

		//Solid Red
		case 'r':
		case 'R':
			userNFCProgrammedAction = RGBRed;
			break;

		//Solid Blue	
		case 'b':
		case 'B':
			userNFCProgrammedAction = RGBBlue;
			break;

		//Solid White
		case 'w':
		case 'W':
			userNFCProgrammedAction = White;
			break;
			
		case 'p':
		case 'P':
			userNFCProgrammedAction = ProgressBar;
			break;
		
		case 'n':
		case 'N':
			userNFCProgrammedAction = NightRider;
			break;
			
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
			userNFCProgrammedAction = keyword_value - 48;	//ASCII to number
			break;
			
		default:
			userNFCProgrammedAction = Idle;
	}
	
	randomSeed(randSeedFromNFC);
}



//Pushbutton handler to detect short or long press
void pushbuttonHandler() {
	if(digitalRead(PB_WAKE) == LOW) {
		//Delay for debounce, 4.7kOhm and 100nF RC debounce circuit -> 	470uSec RC time, x2 (and margin) to get the full rise/fall time without bounces.
		delay(5);
		
		
		if(digitalRead(PB_WAKE) == LOW) {
			//Actually pressed low.
			
			//Check for short or long press in 10mSec increments
			for(char buttonSteps = 0 ; buttonSteps < 10 ; buttonSteps++) {	
				
				if(digitalRead(PB_WAKE)) {
					//Rising edge detected, check for debounce?
					delay(5);
					if(digitalRead(PB_WAKE)) {
						//Short press detected, break
						shortPressDetected = true;
						return;
					}
				}
				
				delay(100);
			
			}
			
			if(digitalRead(PB_WAKE) == LOW) {
				//Long press detected, increment the mode counter
				if(current_mode >= 7) {
					next_mode = 0;
				} else {
					next_mode = current_mode + 1;
				}
				shortPressDetected = false;
				return;
			}
		}

	}
	
	return;
}



//Handles mode changes and indicates to the user the next upcoming mode.
void ModeChange() {
	if(current_mode != next_mode) {
		current_mode = next_mode;

		//Serial.print("[ "); //Serial.print(millis()); //Serial.print(" ] ");
		//Serial.print("Entering mode: ");
		//Serial.println(current_mode, DEC);
		
		charlieplex(current_mode, 1);
		delay(100);
		charlieplex(current_mode, 0);
		delay(100);
		charlieplex(current_mode, 1);
		delay(100);
		charlieplex(current_mode, 0);
		delay(100);
		charlieplex(current_mode, 1);
		delay(100);
		charlieplex(current_mode, 0);
		delay(100);
		
	}
}


//Mode Zero Handler - Multivibrator only mode. Short press action: Toggles on and off Multivibrator
void ModeZero() {
	if(shortPressDetected) {
		shortPressDetected = false;
		digitalWrite(BLINK_DIS, !digitalRead(BLINK_DIS));
	}
}


//Mode One Handler - Short press action: toggles between the different RGB colors on the RGB LED
void ModeOne() {
  digitalWrite(BLINK_DIS, HIGH);
	if(shortPressDetected) {
		shortPressDetected = false;
		if(modeOneStorage >= 2) {
			modeOneStorage = 0;
		} else {
			modeOneStorage++;
		}
	}
  charlieplex(RGBBlue + modeOneStorage, 1);
}


//Mode Two Handler - White LED varied modes. Short press action: toggles between solid white, 1Hz white blink, SOS morse blink then back to solid white
void ModeTwo() {
	digitalWrite(BLINK_DIS, HIGH);
	if(shortPressDetected) {
		shortPressDetected = false;
		if(modeTwoStorage >= 2) {
			modeTwoStorage = 0;
		} else {
			modeTwoStorage++;
		}
	}
		
	switch(modeTwoStorage) {
		default:
			charlieplex(White, 1);
			break;
			
		case 1:
			if((millis() - millisTimestamp) > 500) {
				modeTwoStepCounter = !modeTwoStepCounter;
				millisTimestamp = millis();
				charlieplex(White, modeTwoStepCounter);
			}
			break;
			
		case 2:

			if((millis() - millisTimestamp) > 250) {
				millisTimestamp = millis();
				
				if(modeTwoSOSCounter >= 25) {
					modeTwoSOSCounter = 0;
				} else {
					modeTwoSOSCounter++;
				}
				
				//9 symbols in SOS (. . .##-- -- --##. . .###), dead time between them bumps (spaces) (and 2x for dashes), 2x for intersymbol deadspace (#) too bumps to 26 steps.
				switch(modeTwoSOSCounter) {
					//S dead time
					case 1:
					case 3:

					//Intersymbol dead time
					case 5:
					case 6:
	  
					//O dead time	
					case 9:
					case 12:

					//Intersymbol dead time
					case 15:
					case 16:
					
					//S dead time
					case 18:
					case 20:

					//Intersymbol & end of line dead time
					case 22:
					case 23:
					case 24:
					case 25:
						charlieplex(White, 0);
						break;
					
					//Ons
					default:
						charlieplex(White, 1);
						break;
					
				}
				
			}	
			break;
		
	}

}

//Mode Three Handler - Green LED ring. Short press action: Toggles between Green LED Fast Spinner Clockwise, Green LED Fast Spinner Counter Clockwise, LED ring persistence of vision solid ring and LED ring persistence of vision Binary Counter.
void ModeThree() {
	digitalWrite(BLINK_DIS, HIGH);
	if(shortPressDetected) {
		shortPressDetected = false;
		if(modeThreeStorage >= 3) {
			modeThreeStorage = 0;
		} else {
			modeThreeStorage++;
		}
	}
		
	switch(modeThreeStorage) {
		case 0:
			if((millis() - millisTimestamp) > 125) {
				millisTimestamp = millis();
				
				if(modeThreeSpinnerPos >= 7) {
					modeThreeSpinnerPos = 0;
				} else {
					modeThreeSpinnerPos++;
				}
				charlieplex(modeThreeSpinnerPos,1);
			}
			break;
			
		case 1:
			if((millis() - millisTimestamp) > 125) {
				millisTimestamp = millis();
				
				if(modeThreeSpinnerPos == 0) {
					modeThreeSpinnerPos = 7;
				} else {
					modeThreeSpinnerPos--;
				}
				charlieplex(modeThreeSpinnerPos,1);
			}
			break;
		
		case 2:
			if((millis() - millisTimestamp) > 1) {
				millisTimestamp = millis();
				
				if(modeThreeSpinnerPos == 0) {
					modeThreeSpinnerPos = 7;
				} else {
					modeThreeSpinnerPos--;
				}
				charlieplex(modeThreeSpinnerPos,1);
			}
			break;
		
		case 3:
			if((millis() - millisBinaryCount) > 125) {
				millisBinaryCount = millis();
				modeThreeBinaryCount++;
			}
		
			if((millis() - millisTimestamp) > 1) {
				millisTimestamp = millis();
				
				if(modeThreeSpinnerPos >= 7) {
					modeThreeSpinnerPos = 0;
				} else {
					modeThreeSpinnerPos++;
				}
				charlieplex(modeThreeSpinnerPos, (modeThreeBinaryCount >> modeThreeSpinnerPos) & 1);
			}
			break;
		
		default:
			charlieplex(White, 0);
			break;
		
	}

}


//Mode Four Handler - Compass + Green LED Ring (Mostly uncalibrated)
void ModeFour() {
	shortPressDetected = false;
	digitalWrite(BLINK_DIS, HIGH);
	//LIS3MDL run a conversion
	Wire.beginTransmission(LIS3MDL_addr);
	Wire.write(LIS3MDL_CTRL_REG3);
	Wire.write(0x01); 
	Wire.endTransmission(true);


	delay(125);
	 

	Wire.beginTransmission(LIS3MDL_addr); //Grab X data
	Wire.write(LIS3MDL_OUT_X_L);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		buff = Wire.read(); 
	}

	mag_x = mag_buff;

	Wire.beginTransmission(LIS3MDL_addr); 
	Wire.write(LIS3MDL_OUT_X_H);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		mag_buff = Wire.read(); 
	}

	mag_x = mag_x | (mag_buff << 8);
	mag_x = mag_x - mag_x_offset;



	Wire.beginTransmission(LIS3MDL_addr); //Grab Y data
	Wire.write(LIS3MDL_OUT_Y_L);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		mag_buff = Wire.read(); 
	}

	mag_y = mag_buff;

	Wire.beginTransmission(LIS3MDL_addr); 
	Wire.write(LIS3MDL_OUT_Y_H);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		mag_buff = Wire.read(); 
	}

	mag_y = mag_y | (mag_buff << 8);
	mag_y = mag_y - mag_y_offset;



	Wire.beginTransmission(LIS3MDL_addr); //Grab Y data
	Wire.write(LIS3MDL_OUT_Z_L);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		mag_buff = Wire.read(); 
	}

	mag_z = mag_buff;

	Wire.beginTransmission(LIS3MDL_addr); 
	Wire.write(LIS3MDL_OUT_Z_H);
	Wire.endTransmission(false);

	Wire.requestFrom(LIS3MDL_addr, 1);
	while (Wire.available()) {
		mag_buff = Wire.read(); 
	}

	mag_z = mag_z | (mag_buff << 8);
	mag_z = mag_z - mag_z_offset;
	
	mag_x_double = (double)mag_x / mag_x_scale;
	mag_y_double = (double)mag_y / mag_y_scale;
	//mag_z_double = (double)mag_z / mag_z_scale;


	if(mag_x_double == 0.0) {
		if(mag_y_double < 0.0) {
			angle_double = 90.0;
		} else {
			angle_double = 0.0;
		}
	} else {
		angle_double = atan(mag_y_double / mag_x_double) * rad2deg;
		
		if(angle_double > 360.0) {
			angle_double = angle_double - 360.0;
		} else if (angle_double < 0.0) {
			angle_double = angle_double + 360.0;
		}
	}

	if ((angle < 22.5) || (angle > 337.5)) {
		charlieplex(6,1);
	} else if ((angle < 67.5) && (angle >= 22.5)) {
		charlieplex(5,1);
	} else if ((angle < 112.5) && (angle >= 67.5)) {
		charlieplex(4,1);
	} else if ((angle < 157.5) && (angle >= 112.5)) {
		charlieplex(3,1);
	} else if ((angle < 202.5) && (angle >= 157.5)) {
		charlieplex(2,1);
	} else if ((angle < 247.5) && (angle >= 202.5)) {
		charlieplex(1,1);
	} else if ((angle < 292.5) && (angle >= 247.5)) {
		charlieplex(0,1);
	} else {
		charlieplex(7,1);	
	}	
  
	angle = (int8_t)angle_double;

  
	////Serial.print("X: ");
	//Serial.print(mag_x);
	//Serial.print(",");
	////Serial.print("Y: ");
	//Serial.print(mag_y);
	//Serial.print(",");
	////Serial.print("Z: ");
	//Serial.print(mag_z);
	//Serial.print("  Angle: ");
	//Serial.print(angle);
	//Serial.println();
}




//Mode Five Handler - NFC User Defined LEDs - Short press action: Rereads NFC for new user programmed actions.
void ModeFive() {
	digitalWrite(BLINK_DIS, HIGH);
	if(shortPressDetected) {
		shortPressDetected = false;
		LoadUserNFCConfigurations();
	}
		
	if((userNFCProgrammedAction >= 0) && (userNFCProgrammedAction <= 11)) {
		
		charlieplex(userNFCProgrammedAction,1);
		
	} else {
		
		switch(userNFCProgrammedAction) {
			case ProgressBar:
				if((millis() - millisModeFive) > 250) {
					millisModeFive = millis();
					if(modeFiveProgressBar >= 7) {
						modeFiveProgressBar = 0;
					} else {
						modeFiveProgressBar++;
					}
				}
			
				if((millis() - millisTimestamp) > 1) {
					millisTimestamp = millis();
					
					if(modeFiveStorage >= 7) {
						modeFiveStorage = 0;
					} else {
						modeFiveStorage++;
					}
					
					if(modeFiveStorage <= modeFiveProgressBar) {
						charlieplex(modeFiveStorage, 1);
					} else {
						charlieplex(modeFiveStorage, 0);
					}
				}
				break;
			
			case NightRider:
				if((millis() - millisTimestamp) > 150) {
					millisTimestamp = millis();
					if((modeFiveStorage > 6) || (modeFiveStorage < 2)) {
						modeFiveStorage = 2;					
					}
					if(modeFiveStorage >= 6) {
						modeFiveNightRider = 1;
					} else if (modeFiveStorage <= 2) {
						modeFiveNightRider = 0;
					}
					
					if(modeFiveNightRider == 0) {
						modeFiveStorage++;
					} else {
						modeFiveStorage--;
					}
					
					charlieplex(modeFiveStorage, 1);
				}
				break;
			
			default:
				charlieplex(White, 0);
				break;
			
		}
	}
}


//Mode Six Handler - Random LEDs, randomSeed generated by NFC data at startup & in Mode Five
void ModeSix() {
	shortPressDetected = false;
	if((millis() - millisTimestamp) > 125) {
		millisTimestamp = millis();
		charlieplex(0xFF & random(0,11),1);
	}
}


void WakeUp() {
  sleep_disable();
  detachInterrupt(PB_WAKE);
}





//Arduino Default Setup Block
void setup() {
  BootUp();
  
  LIS3MDL_Setup();
}









//Arduino Default Loop Block
void loop() {
	pinMode(PB_WAKE, INPUT_PULLUP);
	pushbuttonHandler();

	ModeChange();
	
	
	//Different operating modes
	switch(current_mode) {
		case 0:
			ModeZero();
			break;
			
		case 1:
			ModeOne();
			break;
			
		case 2:
			ModeTwo();
			break;
			
		case 3:
			ModeThree();
			break;
		
		case 4:
			ModeFour();
			break;
		
		case 5:
			ModeFive();
			break;
		
		case 6:
			ModeSix();
			break;
			
		
		case 7:
			digitalWrite(BLINK_DIS, HIGH);
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			attachInterrupt(PB_WAKE, WakeUp, LOW);
			charlieplex(White,0);
			sleep_enable();
			sleep_cpu();
			sleep_disable();
			pinMode(PB_WAKE, INPUT_PULLUP);
			delay(5);
			while(digitalRead(PB_WAKE) == LOW) {
				while(digitalRead(PB_WAKE) == LOW) {
					delay(1);
				}
				delay(5);
			}
			delay(25);
			digitalWrite(BLINK_DIS, LOW);
			next_mode = 0;
			break;
		
		default:
			break;
	}


}
