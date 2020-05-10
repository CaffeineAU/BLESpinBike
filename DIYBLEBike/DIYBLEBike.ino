/*********************************************************************
This is an example for our nRF52 based Bluefruit LE modules

Pick one up today in the adafruit shop!

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in
any redistribution
*********************************************************************/
#include <bluefruit.h>

//#include "nrf_timer.h"
//#include "Timer.h"
//
//#define nrf_timer_num   (1)
//#define cc_channel_num  (0)
//TimerClass timer(nrf_timer_num, cc_channel_num);

BLEService        ftms = BLEService(0x1826); //https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.fitness_machine.xml
BLECharacteristic ftmf = BLECharacteristic(0x2ACC); //https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.fitness_machine_feature.xml
BLECharacteristic idbc = BLECharacteristic(0x2AD2); //https://www.bluetooth.com/api/gatt/XmlFile?xmlFileName=org.bluetooth.characteristic.indoor_bike_data.xml 

BLEService        hrms = BLEService(UUID16_SVC_HEART_RATE);
BLECharacteristic hrmc = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLECharacteristic bslc = BLECharacteristic(UUID16_CHR_BODY_SENSOR_LOCATION);

BLEService        spcd = BLEService(UUID16_SVC_CYCLING_SPEED_AND_CADENCE);
BLECharacteristic cscMeas = BLECharacteristic(UUID16_CHR_CSC_MEASUREMENT);
BLECharacteristic cscFeat = BLECharacteristic(UUID16_CHR_CSC_FEATURE);
BLECharacteristic cscLoc = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

BLEService        cypw = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic powMeas = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic powFeat = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic powLoc = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);


BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

uint8_t  bps = 0;


uint32_t loop_count = 0;
uint8_t data_current = 0;
uint8_t data_previous;

int     i2c_start_count = 0;
int     i2c_index = 0;
uint8_t byte_build = 0;
uint8_t byte_index = 0;

bool begin = false;
int packetindex = 0;
uint8_t packet[20];  // buffer to capture a packet


void setup(void)
{

	Serial.begin(115200);
//	while (!Serial) delay(10);   // for nrf52840 with native usb

	Serial.println("Bluefruit52 HRM Example");
	Serial.println("-----------------------\n");

	// Initialise the Bluefruit module
	Serial.println("Initialise the Bluefruit nRF52 module");
	Bluefruit.begin();

	// Set the advertised device name (keep it short!)
	Serial.println("Setting Device Name");
	Bluefruit.setName("Liam's DIY BLE Bike");

	// Set the connect/disconnect callback handlers
	Bluefruit.Periph.setConnectCallback(connect_callback);
	Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

	// Configure and Start the Device Information Service
	Serial.println("Configuring the Device Information Service");
	bledis.setManufacturer("Liam O'Hagan");
	bledis.setModel("GForceUT");
	bledis.begin();

	// Start the BLE Battery Service and set it to 100%
	Serial.println("Configuring the Battery Service");
	blebas.begin();
	blebas.write(100);

	// Setup the services using
	// BLEService and BLECharacteristic classes
	Serial.println("Configuring Services");
	setupHRM();
	//setupSCS();
	setupPOW();
	setupFTMS();

	// Setup the advertising packet(s)
	Serial.println("Setting up the advertising payload(s)");
	startAdv();

	Serial.println("\nAdvertising");

	Serial.println("Configuring inputs");
	pinMode(17, INPUT);
	pinMode(15, INPUT);

	Serial.println("Configuring interrupts");
	attachInterrupt(digitalPinToInterrupt(15), capture_data, CHANGE);
	attachInterrupt(digitalPinToInterrupt(17), capture_data, CHANGE);

}

#define RAWSIZE 16384
uint8_t   raw_read_data[RAWSIZE];  // buffer to capture io pin changes
volatile uint16_t  write_index = 0;
uint16_t  read_index = 0;
volatile uint8_t   current_sda = 0;
volatile uint8_t   current_scl = 0;
volatile uint8_t   last_sda;
volatile uint8_t   last_scl;

// capture_data - this is called by the interrupt whenever the pins change
// When either pin changes state, this routine will capture it in buffer array raw_read_data[]
void capture_data(void)
{
	last_sda = current_sda;
	last_scl = current_scl;

	current_sda = digitalRead(17);
	current_scl = digitalRead(15);
	//Serial.print("SDA ");
	//Serial.print(current_sda);
	//Serial.print("SDA ");
	//Serial.println(current_scl);

	if (current_sda != last_sda || current_scl != last_scl)
	{
//		Serial.println((current_scl * 8) + (current_sda * 4));
		raw_read_data[write_index++] = (current_scl * 8) + (current_sda * 4);
	}
	if (write_index >= RAWSIZE)
		write_index = 0;
}

void startAdv(void)
{
	// Advertising packet
	Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
	Bluefruit.Advertising.addTxPower();

	// Include HRM Service UUID
	Bluefruit.Advertising.addService(ftms, hrms/*, spcd*/, cypw);
	//Bluefruit.Advertising.addService(ftms,hrms/*,spcd,cypw*/);

	// Include Name
	Bluefruit.Advertising.addName();


	/* Start Advertising
	* - Enable auto advertising if disconnected
	* - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
	* - Timeout for fast mode is 30 seconds
	* - Start(timeout) with timeout = 0 will advertise forever (until connected)
	*
	* For recommended advertising interval
	* https://developer.apple.com/library/content/qa/qa1931/_index.html
	*/
	Bluefruit.Advertising.restartOnDisconnect(true);
	Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
	Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
	Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setupHRM(void)
{
	// Configure the Heart Rate Monitor service
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.heart_rate.xml
	// Supported Characteristics:
	// Name                         UUID    Requirement Properties
	// ---------------------------- ------  ----------- ----------
	// Heart Rate Measurement       0x2A37  Mandatory   Notify
	// Body Sensor Location         0x2A38  Optional    Read
	// Heart Rate Control Point     0x2A39  Conditional Write       <-- Not used here
	hrms.begin();

	// Note: You must call .begin() on the BLEService before calling .begin() on
	// any characteristic(s) within that service definition.. Calling .begin() on
	// a BLECharacteristic will cause it to be added to the last BLEService that
	// was 'begin()'ed!

	// Configure the Heart Rate Measurement characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml
	// Properties = Notify
	// Min Len    = 1
	// Max Len    = 8
	//    B0      = UINT8  - Flag (MANDATORY)
	//      b5:7  = Reserved
	//      b4    = RR-Internal (0 = Not present, 1 = Present)
	//      b3    = Energy expended status (0 = Not present, 1 = Present)
	//      b1:2  = Sensor contact status (0+1 = Not supported, 2 = Supported but contact not detected, 3 = Supported and detected)
	//      b0    = Value format (0 = UINT8, 1 = UINT16)
	//    B1      = UINT8  - 8-bit heart rate measurement value in BPM
	//    B2:3    = UINT16 - 16-bit heart rate measurement value in BPM
	//    B4:5    = UINT16 - Energy expended in joules
	//    B6:7    = UINT16 - RR Internal (1/1024 second resolution)
	hrmc.setProperties(CHR_PROPS_NOTIFY);
	hrmc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	hrmc.setFixedLen(2);
	hrmc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
	hrmc.begin();
	uint8_t hrmdata[2] = { 0b00000110, 0x40 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
	hrmc.notify(hrmdata, 2);

	// Configure the Body Sensor Location characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.body_sensor_location.xml
	// Properties = Read
	// Min Len    = 1
	// Max Len    = 1
	//    B0      = UINT8 - Body Sensor Location
	//      0     = Other
	//      1     = Chest
	//      2     = Wrist
	//      3     = Finger
	//      4     = Hand
	//      5     = Ear Lobe
	//      6     = Foot
	//      7:255 = Reserved
	bslc.setProperties(CHR_PROPS_READ);
	bslc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	bslc.setFixedLen(1);
	bslc.begin();
	bslc.write8(2);    // Set the characteristic to 'Wrist' (2)
}

void setupFTMS(void)
{
	// Configure the Fitness Machine service
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.fitness_machine.xml
	// Supported Characteristics:
	// Name                         UUID    Requirement Properties
	// ---------------------------- ------  ----------- ----------
	// Fitness Machine Feature      0x2ACC  Mandatory   Read
	// Indoor Bike Data             0x2AD2  Optional    Notify
	ftms.begin();

	// Note: You must call .begin() on the BLEService before calling .begin() on
	// any characteristic(s) within that service definition.. Calling .begin() on
	// a BLECharacteristic will cause it to be added to the last BLEService that
	// was 'begin()'ed!
	ftmf.setProperties(CHR_PROPS_READ);
	ftmf.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	ftmf.setFixedLen(8);
	ftmf.begin();
	uint8_t ftmfdata[8] = { 0b00000110, 0b01000100, 0b00000000, 0b00000000,0,0,0,0 }; // cadence, total distance, heart rate, power
	ftmf.write(ftmfdata, sizeof(ftmfdata));

	// Configure the Indoor Bike Data characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.indoor_bike_data.xml
	// Properties = Notify
	//    B0      = UINT16  - Flag (MANDATORY)
	//      b0	  = Instantaneous Speed
	//      b1	  = Average Speed
	//      b2	  = Instantaneous Cadence
	//      b3	  = Average Cadence
	//      b4	  = Total Distance
	//      b5	  = Resistance Level
	//      b6	  = Instantaneous Power
	//      b7	  = Average Power
	//      b8	  = Expended Energy
	//      b9	  = Heart Rate
	//      b10	  = Metabolic Equivalent
	//      b11   = Elapsed Time
	//      b12	  = Remaining TIme
	//      b13	  = Reserved
	//      b14	  = Reserved
	//      b15	  = Reserved
	//    B3:4    = UINT16 - Instantaneous speed in km/hr (0.01)
	//    B5:6    = UINT16 - Cadence in 0.5 RPM
	//    B7:9    = UINT24 - Total distance in m
	//    B10:11  = SINT16 - Instantaneousd Power in W
	//    B12     = UINT8  - Heart Rate in bpm

	idbc.setProperties(CHR_PROPS_NOTIFY);
	idbc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	idbc.setFixedLen(12);
	idbc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
	idbc.begin();
	uint8_t hrmdata[12] = { 0b01010101, 0b00000010, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }; // Set the characteristic to have speed, cadence, distance, power, energy and HR
																					   //uint8_t hrmdata[6] = { 0b00000101, 0b00000000, 3, 4, 5, 6 }; // Set the characteristic to have speed, cadence, distance, power and HR
	idbc.notify(hrmdata, sizeof(hrmdata));

	//// Configure the Body Sensor Location characteristic
	//// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.body_sensor_location.xml
	//// Properties = Read
	//// Min Len    = 1
	//// Max Len    = 1
	////    B0      = UINT8 - Body Sensor Location
	////      0     = Other
	////      1     = Chest
	////      2     = Wrist
	////      3     = Finger
	////      4     = Hand
	////      5     = Ear Lobe
	////      6     = Foot
	////      7:255 = Reserved
	//bslc.setProperties(CHR_PROPS_READ);
	//bslc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	//bslc.setFixedLen(1);
	//bslc.begin();
	//bslc.write8(2);    // Set the characteristic to 'Wrist' (2)
}

void setupSCS(void)
{
	// Configure the Cycling Speed and Cadence service
	// See: https://www.bluetooth.com/api/gatt/xmlfile?xmlFileName=org.bluetooth.service.cycling_speed_and_cadence.xml
	// Supported Characteristics:
	// Name                         UUID    Requirement Properties
	// ---------------------------- ------  ----------- ----------
	// CSC Measurement				0x2A5B  Mandatory   Notify
	// CSC Feature		            0x2A5C  Mandatory   Read
	// Sensor Location				0x2A5D  Mandatory   Read
	spcd.begin();

	// Note: You must call .begin() on the BLEService before calling .begin() on
	// any characteristic(s) within that service definition.. Calling .begin() on
	// a BLECharacteristic will cause it to be added to the last BLEService that
	// was 'begin()'ed!

	// Configure the CSC Measurement characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.csc_measurement.xml
	// Properties = Notify
	//    B0      = UINT8  - Flag (MANDATORY)
	//      b0	  = Wheel Revolution Data Present
	//      b1	  = Crank Revolution Data Present
	//      b2	  = Reserved
	//      b3	  = Reserved
	//      b4	  = Reserved
	//      b5	  = Reserved
	//      b6	  = Reserved
	//      b7	  = Reserved
	//    B1:4    = UINT16  - Cumulative Crank Revolutions
	//    B5:6    = UINT16 - Last Crank Event Time (1/1024 seconds)
	cscMeas.setProperties(CHR_PROPS_NOTIFY);
	cscMeas.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	cscMeas.setFixedLen(5);
	cscMeas.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
	cscMeas.begin();
	uint8_t measdata[5] = { 0b00000010, 0, 0, 0, 0 };
	cscMeas.notify(measdata, sizeof(measdata));

	//// Configure the CSC Feature characteristic
	//// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.csc_feature.xml
	//// Properties = Read
	////    B0      = UINT8 - CSC Feature
	////      b0    = Wheel Revolution Data Supported
	////      b1    = Crank Revolution Data Supported
	////      b2    = Multiple Sensor Locations Supported
	////      b3:b15= Reserved
	cscFeat.setProperties(CHR_PROPS_READ);
	cscFeat.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	cscFeat.setFixedLen(2);
	cscFeat.begin();
	cscFeat.write16(2);    // Set the characteristic to Crank Revolution Data Supported

						   // Configure the Sensor Location characteristic
						   // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
						   // Properties = Read
						   //    B0      = UINT8  - Enumeration (MANDATORY)
						   //      0	  = Other
						   //      1	  = Top of shoe
						   //      2	  = In shoe
						   //      3	  = Hip
						   //      4	  = Front Wheel
						   //      5	  = Left Crank
						   //      6	  = Right Crank
						   //      7	  = Left Pedal
						   //      8	  = Right Pedal
						   //      9	  = Front Hub
						   //      10	  = Rear Dropout
						   //      11	  = Chainstay
						   //      12	  = Rear Wheel
						   //      13	  = Rear Hub
						   //      14	  = Chest
						   //      15	  = Spider
						   //      16	  = Chain Ring	
						   //		17-255= Reserved
	cscLoc.setProperties(CHR_PROPS_READ);
	cscLoc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	cscLoc.setFixedLen(1);
	cscLoc.begin();
	cscLoc.write8(0b00100000); // Left crank
}

void setupPOW(void)
{
	// Configure the Cycling Power service
	// See: https://www.bluetooth.com/api/gatt/xmlfile?xmlFileName=org.bluetooth.service.cycling_power.xml
	// Supported Characteristics:
	// Name                         UUID    Requirement Properties
	// ---------------------------- ------  ----------- ----------
	// Cycling Power Measurement    0x2A63  Mandatory   Notify
	// Cycling Power Feature        0x2AD2  Mandatory   Read
	// Sensor Location				0x2A5D  Mandatory   Read
	cypw.begin();

	// Note: You must call .begin() on the BLEService before calling .begin() on
	// any characteristic(s) within that service definition.. Calling .begin() on
	// a BLECharacteristic will cause it to be added to the last BLEService that
	// was 'begin()'ed!

	// Configure the Cycling Power Measurement characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_measurement.xml
	// Properties = Notify
	//    B0      = UINT16  - Flag (MANDATORY)
	//      b0	  = Pedal Power Balance Present
	//      b1	  = Pedal Power Balance Reference
	//      b2	  = Accumulated Torque Present
	//      b3	  = Accumulated Torque Source
	//      b4	  = Wheel Revolution Data Present
	//      b5	  = Crank Revolution Data Present
	//      b6	  = Extreme Force Magnitudes Present
	//      b7	  = Extreme Torque Magnitudes Present 
	//      b8	  = Extreme Angles Present
	//      b9	  = Top Dead Spot Angle Present
	//      b10	  = Bottom Dead Spot Angle Present
	//      b11   = Accumulated Energy Present
	//      b12	  = Offset Compensation Indicator
	//      b13	  = Reserved
	//      b14	  = Reserved
	//      b15	  = Reserved
	//    B1:2      = SINT16 - Instantaneous Power
	powMeas.setProperties(CHR_PROPS_NOTIFY);
	powMeas.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	powMeas.setFixedLen(8);
	powMeas.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
	powMeas.begin();
	uint8_t powdat[8] = { 0b00100000, 0b00000000, 1, 2, 3, 4 }; // Crank Revolution Supported
	powMeas.notify(powdat, sizeof(powdat));

	// Configure the Cycling Power Feature characteristic
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.cycling_power_feature.xml
	// Properties = REad
	//    B0      = UINT32  - Flag (MANDATORY)
	//      b0	  = Pedal Power Balance Supported
	//      b1	  = Accumulated Torque Supported
	//      b2	  = Wheel Revolution Data Supported
	//      b3	  = Crank Revolution Data Supported
	//      b4	  = Extreme Magnitudes Supported
	//      b5	  = Extreme Angles Supported
	//      b6	  = Top and Bottom Dead Spot Angles Supported
	//      b7	  = Accumulated Energy Supported
	//      b8	  = Offset Compensation Indicator Supported
	//      b9	  = Offset Compensation Supported
	//      b10	  = Cycling Power Measurement Characteristic Content Masking Supported
	//      b11   = Multiple Sensor Locations Supported
	//      b12	  = Crank Length Adjustment Supported
	//      b13	  = Chain Length Adjustment Supported
	//      b14	  = Chain Weight Adjustment Supported
	//      b15	  = Span Length Adjustment Supported
	//      b16	  = Sensor Measurement Context (0=force, 1 = torque)
	//      b17	  = Instantaneous Measurement Direction Supported
	//      b18	  = Factory Calibration Date Supported
	//      b19	  = Enhanced Offset Compensation Supported
	//      b20:21= Distribute System Support (0=Unspecified (legacy sensor), 1=Not for use in a distributed system, 2=Can be used in a distributed system, 3=RFU)
	//      b22:31= Reserved
	//    B1:2      = SINT16 - Instantaneous Power
	powFeat.setProperties(CHR_PROPS_READ);
	powFeat.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	powFeat.setFixedLen(4);
	powFeat.begin();
	powFeat.write32(0b1000); // Crank Revolution Data Supported

							 // Configure the Sensor Location characteristic
							 // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
							 // Properties = Read
							 //    B0      = UINT8  - Enumeration (MANDATORY)
							 //      0	  = Other
							 //      1	  = Top of shoe
							 //      2	  = In shoe
							 //      3	  = Hip
							 //      4	  = Front Wheel
							 //      5	  = Left Crank
							 //      6	  = Right Crank
							 //      7	  = Left Pedal
							 //      8	  = Right Pedal
							 //      9	  = Front Hub
							 //      10	  = Rear Dropout
							 //      11	  = Chainstay
							 //      12	  = Rear Wheel
							 //      13	  = Rear Hub
							 //      14	  = Chest
							 //      15	  = Spider
							 //      16	  = Chain Ring	
							 //		17-255= Reserved
	powLoc.setProperties(CHR_PROPS_READ);
	powLoc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
	powLoc.setFixedLen(1);
	powLoc.begin();
	powLoc.write8(0b00100000); // Left crank
}

void connect_callback(uint16_t conn_handle)
{
	// Get the reference to current connection
	BLEConnection* connection = Bluefruit.Connection(conn_handle);

	char central_name[32] = { 0 };
	connection->getPeerName(central_name, sizeof(central_name));

	Serial.print("Connected to ");
	Serial.println(central_name);
}

/**
* Callback invoked when a connection is dropped
* @param conn_handle connection where this event happens
* @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
	(void)conn_handle;
	(void)reason;

	Serial.println("Disconnected");
	Serial.println("Advertising!");
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
	// Display the raw request packet
	Serial.print("CCCD Updated: ");
	//Serial.printBuffer(request->data, request->len);
	Serial.print(cccd_value);
	Serial.println("");

	// Check the characteristic this CCCD update is associated with in case
	// this handler is used for multiple CCCD records.
	if (chr->uuid == idbc.uuid) {
		if (chr->notifyEnabled(conn_hdl)) {
			Serial.println("Indoor Bike 'Notify' enabled");
		}
		else {
			Serial.println("Indoor Bike 'Notify' disabled");
		}
	}
	// Check the characteristic this CCCD update is associated with in case
	// this handler is used for multiple CCCD records.
	if (chr->uuid == hrmc.uuid) {
		if (chr->notifyEnabled(conn_hdl)) {
			Serial.println("Heart Rate Measurement 'Notify' enabled");
		}
		else {
			Serial.println("Heart Rate Measurement 'Notify' disabled");
		}
	}
	// Check the characteristic this CCCD update is associated with in case
	// this handler is used for multiple CCCD records.
	if (chr->uuid == cscMeas.uuid) {
		if (chr->notifyEnabled(conn_hdl)) {
			Serial.println("CSC Measurement 'Notify' enabled");
		}
		else {
			Serial.println("CSC Measurement 'Notify' disabled");
		}
	}
	// Check the characteristic this CCCD update is associated with in case
	// this handler is used for multiple CCCD records.
	if (chr->uuid == powMeas.uuid) {
		if (chr->notifyEnabled(conn_hdl)) {
			Serial.println("Power Measurement 'Notify' enabled");
		}
		else {
			Serial.println("Power Measurement 'Notify' disabled");
		}
	}
}

uint8_t Speed[2]={ 0,0 };
uint8_t Distance[2] = { 0,0 };
uint8_t Power[2] = { 0,0 };
uint8_t HR = 0;
uint8_t RPM = 0;
uint8_t Calories[2] = { 0,0 };



void loop()
{

	//if (Bluefruit.connected())
	//{
	//	uint8_t ibsdata[12] = { 0b01010100, 0b00000010, Speed[1], Speed[0], RPM*2, 0, Distance[1], Distance[0], 0, Power[1], Power[0], HR}; // Set the characteristic to have speed, cadence, distance, power, energy and HR
	//	if (idbc.notify(ibsdata, sizeof(ibsdata)))
	//	{
	//		Serial.print("Indoor Bike updated to: "); Serial.println(bps);
	//	}
	//	else
	//	{
	//		Serial.println("ERROR: IBS Notify not set in the CCCD or not connected!");
	//	}

	//	uint8_t hrmdata[2] = { 0b00000110, bps++ };           // Sensor connected, increment BPS value
	//	if (hrmc.notify(hrmdata, sizeof(hrmdata)))
	//	{
	//		Serial.print("Heart Rate Measurement updated to: "); Serial.println(bps);
	//	}
	//	else
	//	{
	//		Serial.println("ERROR: HR Notify not set in the CCCD or not connected!");
	//	}

	//	uint8_t powdat[8] = { 0b00000000, 0b00000000, bps,1, bps,0, 0, 2 }; // Set the characteristic to have power, crank revs and crank timing
	//	if (powMeas.notify(powdat, sizeof(powdat)))
	//	{
	//		Serial.print("Power Measurement updated to: "); Serial.println(bps);
	//	}
	//	else
	//	{
	//		Serial.println("ERROR: Power Notify not set in the CCCD or not connected!");
	//	}

	//	uint8_t measdata[5] = { 0b00000010, bps,0, 0, 2 };
	//	if (cscMeas.notify(measdata, sizeof(measdata)))
	//	{
	//		Serial.print("Cadence Measurement updated to: "); Serial.println(bps);
	//	}
	//	else
	//	{
	//		Serial.println("ERROR: CSC Notify not set in the CCCD or not connected!");
	//	}

	//}

	// Only send update once per second
	bool bDataReady;
	noInterrupts();
	bDataReady = (read_index != write_index);
	interrupts();


	if (bDataReady)
	{
		data_previous = data_current;
		data_current = raw_read_data[read_index++];
		if (read_index >= RAWSIZE)
			read_index = 0;
		if ((data_previous == 0xC) && (data_current == 0x8))
		{
			//if (i2c_start_count)
			//	Serial.print("Sr,");
			//else
			//	Serial.print("S,");
			packetindex = 0;
			byte_build = 0;
			byte_index = 0;
			i2c_index = 0;
			i2c_start_count++;
		}
		else if ((data_previous == 0x8) && (data_current == 0xC))
		{
			//Serial.println("P");
			//Serial.printf("Packet = ");
			//for (uint8_t i = 0; i <= packetindex; i++)
			//{
			//	Serial.printf("0x%02X,", byte_build);
			//}
			//Serial.println("");
			if (packet[1] == 0x05)
			{
				uint16_t speed = packet[3] * 255 + packet[4];
				speed *= 10;
				Speed[1] = speed & 0xff;
				Speed[0] = (speed >> 8);
				uint16_t distance = packet[5] * 255 + packet[6];
				distance *= 100;
				Distance[1] = distance & 0xff;
				Distance[0] = (distance >> 8);
				Power[0] = packet[7];
				Power[1] = packet[8];
				HR = packet[9];
				RPM = packet[11];
				Calories[0] = packet[12];
				Calories[1] = packet[13];

				//Serial.printf("Speed = %4.1f, ", (float)(Speed[0] * 255 + Speed[1]) / 10.0);
				//Serial.printf("Distance = %4.1f, ", (float)(Distance[0] * 255 + Distance[1]) / 10.0);
				//Serial.printf("Power = %d, ", Power[0] * 255 + Power[1]);
				//Serial.printf("HR = %d, ", HR);
				//Serial.printf("RPM = %d, ", RPM);
				//Serial.printf("Calories = %d, ", Calories[0] * 255 + Calories[1]);
				//Serial.println("");
				if (Bluefruit.connected())
				{
					uint8_t ibsdata[12] = { 0b01010100, 0b00000010, Speed[1], Speed[0], RPM * 2, 0, Distance[1], Distance[0], 0, Power[1], Power[0], HR }; // Set the characteristic to have speed, cadence, distance, power, energy and HR
					if (idbc.notify(ibsdata, sizeof(ibsdata)))
					{
						Serial.print("Indoor Bike updated to: "); Serial.println(bps);
					}
					else
					{
						Serial.println("ERROR: IBS Notify not set in the CCCD or not connected!");
					}

					uint8_t hrmdata[2] = { 0b00000110, HR };           // Sensor connected, increment BPS value
					if (hrmc.notify(hrmdata, sizeof(hrmdata)))
					{
						Serial.print("Heart Rate Measurement updated to: "); Serial.println(bps);
					}
					else
					{
						Serial.println("ERROR: HR Notify not set in the CCCD or not connected!");
					}

					uint8_t powdat[8] = { 0b00000000, 0b00000000, Power[1], Power[0], RPM,0, 0, 2 }; // Set the characteristic to have power, crank revs and crank timing
					if (powMeas.notify(powdat, sizeof(powdat)))
					{
						Serial.print("Power Measurement updated to: "); Serial.println(bps);
					}
					else
					{
						Serial.println("ERROR: Power Notify not set in the CCCD or not connected!");
					}

				}
			}
			begin = false;
			i2c_start_count = 0;
		}
		else if ((data_previous == 0x4) && (data_current == 0xC))
		{	// ONE 
			if (i2c_index <= 6)
				byte_build |= 1 << (6 - byte_index);  // BUILD ADDRESS
			else
				byte_build |= 1 << (7 - byte_index % 8);  // BUILD BYTE
			byte_index++;
			i2c_index++;
		}
		else if ((data_previous == 0x0) && (data_current == 0x8))
		{
			// ZERO
			byte_index++;
			i2c_index++;
		}
		// print stuff out
		if ((i2c_index == 7) && (byte_index == 7))
		{
			//Serial.printf("Addr=0x%02X,", byte_build);
			packet[packetindex++] = byte_build;
			byte_build = 0;
			byte_index = 0;
		}
		else if ((i2c_index == 8) && (byte_index == 1))
		{
			//if (byte_build)
			//	Serial.print("R,");
			//else
			//	Serial.print("W,");
			byte_build = 0;
			byte_index = 0;
		}
		else if ((i2c_index == 9) && (byte_index == 1))
		{
			//if (byte_build)
			//	Serial.print("N,");
			//else
			//	Serial.print("A,");
			byte_build = 0;
			byte_index = 0;
		}
		else if ((i2c_index == 17) && (byte_index == 8) && (i2c_start_count == 1))
		{
			//Serial.printf("%s(0x%02X),", Cmd2Str(byte_build), byte_build);
			//Serial.printf("0x%02X,", byte_build);
			packet[packetindex++] = byte_build;

			byte_build = 0;

			byte_index = 9; // still waiting for ACK but force index change to prevent double printing
		}
		else
		{
			if (byte_index == 8)
			{
				//Serial.printf("0x%02X,", byte_build);
				packet[packetindex++] = byte_build;

				byte_build = 0;
				byte_index = 9; // still waiting for ACK but force index change to prevent double printing
			}
			else if (byte_index == 10)
			{
				//if (byte_build)
				//	Serial.print("N,");
				//else
				//	Serial.print("A,");
				byte_build = 0;
				byte_index = 0;
			}
		}
	}
	//else
	//{
	//	if (Bluefruit.connected())
	//	{
	//		uint8_t ibsdata[12] = { 0b01010101, 0b00000010, Speed[1], Speed[0], RPM * 2, 0, Distance[1], Distance[0], 0, Power[1], Power[0], HR }; // Set the characteristic to have speed, cadence, distance, power, energy and HR
	//		if (idbc.notify(ibsdata, sizeof(ibsdata)))
	//		{
	//			Serial.print("Indoor Bike updated to: "); Serial.println(bps);
	//		}
	//		else
	//		{
	//			Serial.println("ERROR: IBS Notify not set in the CCCD or not connected!");
	//		}

	//		uint8_t hrmdata[2] = { 0b00000110, HR };           // Sensor connected, increment BPS value
	//		if (hrmc.notify(hrmdata, sizeof(hrmdata)))
	//		{
	//			Serial.print("Heart Rate Measurement updated to: "); Serial.println(bps);
	//		}
	//		else
	//		{
	//			Serial.println("ERROR: HR Notify not set in the CCCD or not connected!");
	//		}

	//		uint8_t powdat[8] = { 0b00000000, 0b00000000, Power[1], Power[0], RPM,0, 0, 2 }; // Set the characteristic to have power, crank revs and crank timing
	//		if (powMeas.notify(powdat, sizeof(powdat)))
	//		{
	//			Serial.print("Power Measurement updated to: "); Serial.println(bps);
	//		}
	//		else
	//		{
	//			Serial.println("ERROR: Power Notify not set in the CCCD or not connected!");
	//		}

	//	}

	//}
	//else
	//	if ((i2c_start_count == 0) && (loop_count++ % 500000 == 0))
	//		Serial.println("Waiting for data...");
	//delay(500);


}