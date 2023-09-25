/*
Connect to the LTC6804 and do some testing
Written by Anthony Miceli 
Last Modified 3/28/18

This script runs on a LINEAR ONE, which is similar to an Arduino uno. 
It will not work on an arduino uno as-is. 
Uses Seeed CAN-BUS shield v1.2
*/

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC68041.h"
#include <SPI.h>
#include <mcp_can.h>

// Global Vars
const int NUM_ASICS = 1;  // number of ASICs in daisy chain
const int NUM_CELLS = 12; // battery cells to be read
const int SPI_CS_PIN = 9;

int ASIC = 1;
const int LTC6804 = 1;
const int I2CCHIP = 2;
uint8_t tx_cfg[NUM_ASICS][6];
uint16_t cell_codes[NUM_ASICS][12];


const uint16_t MAX_V_LTC = 50000;
const uint16_t MIN_V_LTC = 7000;
bool ascendingSweep = true;
unsigned char stmp[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
unsigned char bloomV[2] = { 0, 0 };
uint16_t bloomVunscaled = 7000; //start at .7 V  (10000 == 1 V) 
const int STEPSIZE = 1000; //increment of .1 V
bool testComplete = false; 
uint16_t celldata[NUM_CELLS];
unsigned long currentMs = millis();
int evenOdd = 0; // Bloomy V changes only occur on odd loops.



MCP_CAN CAN(SPI_CS_PIN); // Set CS pin



void setup()
{
  Serial.begin(115200);
  init_cfg();
  spi_enable(SPI_CLOCK_DIV16);
  delay(100);
  //Serial.println("CAN BUS Shield init ok!");

  // init CAN for bloomy setup
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) {}
  
  // Cell current set all to 200 mA
  unsigned char bloomI[4] = { 208, 7, 208, 7};
  CAN.sendMsgBuf(0x481, 0, 4, bloomI);
  
  //set voltage to 0 initally
  CAN.sendMsgBuf(0x501, 0, 2, bloomV);
  
  unsigned char enable[1] = {1};
  // enable all bloomy cells
  CAN.sendMsgBuf(0x541, 0, 1, enable);
}

   // set initial voltage values
   //bitwise shift, and  'and' with 0xFF as a logic mask
  bloomV[1] = char(bloomVunscaled >> 8); //btw the endianness of bloomy is weird
  bloomV[0] = char(bloomVunscaled & 0xFF);
  CAN.sendMsgBuf(0x501, 0, 2, bloomV);
}


  /* 
  int ASIC = determineASIC();
  */
  



void loop()
{
  while (!testComplete) // stop looping after complete
  { 
	wakeup_sleep();
	LTC6804_wrcfg(NUM_ASICS, tx_cfg);
	wakeup_idle();
	LTC6804_adcv();
	wakeup_idle();
	int8_t error = 0;
	error = LTC6804_rdcv(0, NUM_ASICS, cell_codes);
	if((evenOdd % 2) == 1)
	{
		// This is the logic to run the test every second.
		// change this number to change interval time
		if(millis() - currentMs <= 990)
		{
			print_cells();  //read asic data
			dataSend(); //process and send data
		}
		else 
		{
			sweepTest();
			//randomStepTest();
		}
	}
	print_cells();  //read asic data
	dataSend(); //process and send data
	if(evenOdd >= 1000)
	{
		evenOdd = 0;
	}
  evenOdd = evenOdd + 1;
  LTC6804_initialize(); 
  }
}

/*************************************************************
*  This loops through every cell reading from the asic
*  Processes the data in a compact format for vehiclespy 
*  12 cells' data can fit into 3 CAN messages. Dynamic loop
*  that adjusts based on the NUM_CELLS. Sends CAN msg when full.
*************************************************************/
void dataSend()
{
	for (int count = 0; count < NUM_CELLS; count++){
		uint16_t cell = (celldata[count]);
		switch(count % 4)
		{
		case 0:
		{
			stmp[0] = char(cell >> 8);
			stmp[1] = char(cell & 0xFF);
			break;
		}
		case 1:
		{
			stmp[2] = char(cell >> 8);
			stmp[3] = char(cell & 0xFF);
			break;
		}
		case 2:
		{
			stmp[4] = char(cell >> 8);
			stmp[5] = char(cell & 0xFF);
			break;
		}
		case 3:
		{
			stmp[6] = char(cell >> 8);
			stmp[7] = char(cell & 0xFF);
			break;
		}
		}
		 if ((count + 1) % 4 == 0 || (count + 1) == NUM_CELLS){
        CAN.sendMsgBuf((count - 3), 0, 8, stmp);
      }
	}
}
/*************************************************************
*  Test that goes from .7V to 5V, then back down to .7V
*  Uncomment the test complete to have it only run once.
*************************************************************/
void sweepTest()
{
	currentMs = millis();
	if((bloomVunscaled < MAX_V_LTC) && ascendingSweep)
	{
		CAN.sendMsgBuf(0x501, 0, 2, bloomV);
		calcBloomV(ascendingSweep); 
		ascendingSweep = (bloomVunscaled >= MAX_V_LTC) ? false : true;
	}
	if (!ascendingSweep && bloomVunscaled < 5000){
		ascendingSweep = true;
	}
	//descending sweep
	if ((bloomVunscaled > MIN_V_LTC) && !ascendingSweep)
	{
		CAN.sendMsgBuf(0x501, 0, 2, bloomV);
		calcBloomV(false);
	}
	else {
	//testComplete = true;
	}
}
/*************************************************************
*  This calculates the format needed for CAN msgs to control bloomy.
*  To be used for comparisons
*************************************************************/
void calcBloomV(bool ascend)
{
	bloomVunscaled = (ascend) ? 
		bloomVunscaled+STEPSIZE : bloomVunscaled-STEPSIZE;
		//search "ternary operator" for more information
  
	  //shift right 8 bits to get most significant bits
	  bloomV[1] = char(bloomVunscaled >> 8);
	  //bitwise 'and' with 0xFF as a logic mask
	  bloomV[0] = char(bloomVunscaled & 0xFF);
}

/*************************************************************
*  Choose a random value between .7 and 5 V to set bloomy
*  Working as of 3/26/18
*  TODO: implement an arbitrary amount of steps to test
*************************************************************/
void randomStepTest()
{
	
	currentMs = millis();
	int randStep = random(7,50);
	unsigned char bloomVRandom[2];
	
	randStep = randStep*1000;
	
	bloomVRandom[0] = char(randStep & 0xFF);
	bloomVRandom[1] = char(randStep >> 8);

	CAN.sendMsgBuf(0x501, 0, 2, bloomVRandom);
}
/*************************************************************
*  This grabs data from the LTC68041 chip.
*  Uncomment the comments to get a serial printout of voltages
*************************************************************/
void print_cells()
{
	for (int current_ic = 0; current_ic < NUM_ASICS; current_ic++)
	{     
		for (int i = 0; i < NUM_CELLS; i++)
		{
			celldata[i] = cell_codes[current_ic][i];
		}
	}
}
void init_cfg()
{
  for (int i = 0; i < NUM_ASICS; i++)
  {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00;
    tx_cfg[i][2] = 0x00;
    tx_cfg[i][3] = 0x00;
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x00;
  }
}

