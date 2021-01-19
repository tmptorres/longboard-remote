/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len);

int ReceiveUartMessage(uint8_t* payloadReceived) {

	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
	 
	int counter = 0;
	int endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	int lenPayload = 0;

	while (SERIALIO.available()) {

		messageReceived[counter++] = SERIALIO.read();

		if (counter == 2) {//case if state of 'counter' with last read 1

			switch (messageReceived[0])
			{
			case 2:
				endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
				lenPayload = messageReceived[1];
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}

		}
		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
			messageReceived[endMessage] = 0;
#ifdef DEBUG
			DEBUGSERIAL.println("End of message reached!");
#endif			
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer. 
		}
	}
	bool unpacked = false;
	if (messageRead) {
		unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read

	}
	else {
		return 0; //No Message Read
	}
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];
#ifdef DEBUG
	DEBUGSERIAL.print("SRC received: "); DEBUGSERIAL.println(crcMessage);
#endif // DEBUG

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
#ifdef DEBUG
	DEBUGSERIAL.print("SRC calc: "); DEBUGSERIAL.println(crcPayload);
#endif
	if (crcPayload == crcMessage)
	{
#ifdef DEBUG
		DEBUGSERIAL.print("Received: "); SerialPrint(message, lenMes); DEBUGSERIAL.println();
		DEBUGSERIAL.print("Payload :      "); SerialPrint(payload, message[1] - 1); DEBUGSERIAL.println();
#endif // DEBUG

		return true;
	}
	else
	{
		return false;
	}
}

int PackSendPayload(uint8_t* payload, int lenPay) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

#ifdef DEBUG
	DEBUGSERIAL.print("UART package send: "); SerialPrint(messageSend, count);

#endif // DEBUG

	//Sending package
	SERIALIO.write(messageSend, count);


	//Returns number of send bytes
	return count;
}


bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_VALUES:
		// ind = 14; //Skipped the first 14 bit.
		// values.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
		// values.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
		// values.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
		// values.rpm = buffer_get_int32(message, &ind);
		// values.inpVoltage = buffer_get_float16(message, 10.0, &ind);
		// values.ampHours = buffer_get_float32(message, 10000.0, &ind);
		// values.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
		// ind += 8; //Skip 9 bit
		// values.tachometer = buffer_get_int32(message, &ind);
		// values.tachometerAbs = buffer_get_int32(message, &ind);

		// My Read
		values.temp_mos = buffer_get_float16(message, 1e1, &ind);
		values.temp_motor = buffer_get_float16(message, 1e1, &ind);
		values.current_motor = buffer_get_float32(message, 1e2, &ind);
		values.current_in = buffer_get_float32(message, 1e2, &ind);
		values.id = buffer_get_float32(message, 1e2, &ind);
		values.iq = buffer_get_float32(message, 1e2, &ind);
		values.duty_now = buffer_get_float16(message, 1e3, &ind);
		values.rpm = buffer_get_float32(message, 1e0, &ind);
		values.v_in = buffer_get_float16(message, 1e1, &ind);
		values.amp_hours = buffer_get_float32(message, 1e4, &ind);
		values.amp_hours_charged = buffer_get_float32(message, 1e4, &ind);
		values.watt_hours = buffer_get_float32(message, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(message, 1e4, &ind);
		values.tachometer = buffer_get_int32(message, &ind);
		values.tachometer_abs = buffer_get_int32(message, &ind);
		values.fault_code = (mc_fault_code) message[ind++];

		if ( len - ind >= 4) {
            values.position = buffer_get_float32(message, 1e6, &ind);
        } else {
            values.position = -1.0;
        }

	default:
		return false;
		break;
	}

}

bool VescUartGetValue(bldcMeasure& values) {
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];
	PackSendPayload(command, 1);
	delay(100); //needed, otherwise data is not read
	int lenPayload = ReceiveUartMessage(payload);
	if (lenPayload > 55) {
		bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

void VescUartSetCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5);
}

void VescUartSetCurrentBrake(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	PackSendPayload(payload, 5);

}

void VescUartSetNunchukValues(remotePackage& data) {
	int32_t ind = 0;
	uint8_t payload[11];
	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = data.valXJoy;
	payload[ind++] = data.valYJoy;
	buffer_append_bool(payload, data.valLowerButton, &ind);
	buffer_append_bool(payload, data.valUpperButton, &ind);
	//Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

#ifdef DEBUG
	DEBUGSERIAL.println("Data reached at VescUartSetNunchuckValues:");
	DEBUGSERIAL.print("valXJoy = "); DEBUGSERIAL.print(data.valXJoy); DEBUGSERIAL.print(" valYJoy = "); DEBUGSERIAL.println(data.valYJoy);
	DEBUGSERIAL.print("LowerButton = "); DEBUGSERIAL.print(data.valLowerButton); DEBUGSERIAL.print(" UpperButton = "); DEBUGSERIAL.println(data.valUpperButton);
#endif

	PackSendPayload(payload, 11);
}

void SerialPrint(uint8_t* data, int len) {

	//	DEBUGSERIAL.print("Data to display: "); DEBUGSERIAL.println(sizeof(data));

	for (int i = 0; i <= len; i++)
	{
		DEBUGSERIAL.print(data[i]);
		DEBUGSERIAL.print(" ");
	}
	DEBUGSERIAL.println("");
}

// Helpers

// bldc_interface_fault_to_string was forked from:
// https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery/blob/master/bldc_interface.c

const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE"; break;
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE"; break;
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV"; break;
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT"; break;
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET"; break;
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR"; break;
	default: return "Unknown fault"; break;
	}
}

void SerialPrint(const bldcMeasure& values) {
	// DEBUGSERIAL.print("avgMotorCurrent: "); DEBUGSERIAL.println(values.avgMotorCurrent);
	// DEBUGSERIAL.print("avgInputCurrent: "); DEBUGSERIAL.println(values.avgInputCurrent);
	// DEBUGSERIAL.print("dutyCycleNow: "); DEBUGSERIAL.println(values.dutyCycleNow);
	// DEBUGSERIAL.print("rpm: "); DEBUGSERIAL.println(values.rpm);
	// DEBUGSERIAL.print("inputVoltage: "); DEBUGSERIAL.println(values.inpVoltage);
	// DEBUGSERIAL.print("ampHours: "); DEBUGSERIAL.println(values.ampHours);
	// DEBUGSERIAL.print("ampHoursCharges: "); DEBUGSERIAL.println(values.ampHoursCharged);
	// DEBUGSERIAL.print("tachometer: "); DEBUGSERIAL.println(values.tachometer);
	// DEBUGSERIAL.print("tachometerAbs: "); DEBUGSERIAL.println(values.tachometerAbs);


	// Commented out to use Vedder's struct
	// DEBUGSERIAL.print("v_in: 				"); DEBUGSERIAL.println(values.v_in);
	// DEBUGSERIAL.print("temp_mos1: 			"); DEBUGSERIAL.println(values.temp_mos1);
	// DEBUGSERIAL.print("temp_mos2: 			"); DEBUGSERIAL.println(values.temp_mos2);
	// DEBUGSERIAL.print("temp_mos3: 			"); DEBUGSERIAL.println(values.temp_mos3);
	// DEBUGSERIAL.print("temp_mos4: 			"); DEBUGSERIAL.println(values.temp_mos4);
	// DEBUGSERIAL.print("temp_mos5: 			"); DEBUGSERIAL.println(values.temp_mos5);
	// DEBUGSERIAL.print("temp_mos6: 			"); DEBUGSERIAL.println(values.temp_mos6);
	// DEBUGSERIAL.print("temp_pcb: 			"); DEBUGSERIAL.println(values.temp_pcb);
	// DEBUGSERIAL.print("current_motor: 		"); DEBUGSERIAL.println(values.current_motor);
	// DEBUGSERIAL.print("current_in: 			"); DEBUGSERIAL.println(values.current_in);
	// DEBUGSERIAL.print("erpm: 				"); DEBUGSERIAL.println(values.rpm);
	// DEBUGSERIAL.print("duty_now: 			"); DEBUGSERIAL.println(values.duty_now);
	// DEBUGSERIAL.print("amp_hours: 			"); DEBUGSERIAL.println(values.amp_hours);
	// DEBUGSERIAL.print("amp_hours_charged: 	"); DEBUGSERIAL.println(values.amp_hours_charged);
	// DEBUGSERIAL.print("watt_hours: 			"); DEBUGSERIAL.println(values.watt_hours);
	// DEBUGSERIAL.print("watt_hours_charged: 	"); DEBUGSERIAL.println(values.watt_hours_charged);
	// DEBUGSERIAL.print("tachometer: 			"); DEBUGSERIAL.println(values.tachometer);
	// DEBUGSERIAL.print("tachometer_abs: 		"); DEBUGSERIAL.println(values.tachometer_abs);
	// DEBUGSERIAL.print("fault_code: 			"); DEBUGSERIAL.println(bldc_interface_fault_to_string(values.fault_code));

	// Commented out to use Vedder's struct
	DEBUGSERIAL.print("temp_mos: 			"); DEBUGSERIAL.println(values.temp_mos);
	DEBUGSERIAL.print("temp_motor: 			"); DEBUGSERIAL.println(values.temp_motor);
	DEBUGSERIAL.print("current_motor: 		"); DEBUGSERIAL.println(values.current_motor);
	DEBUGSERIAL.print("current_in: 			"); DEBUGSERIAL.println(values.current_in);
	DEBUGSERIAL.print("id:	 				"); DEBUGSERIAL.println(values.id);
	DEBUGSERIAL.print("iq:	 				"); DEBUGSERIAL.println(values.iq);
	DEBUGSERIAL.print("erpm: 				"); DEBUGSERIAL.println(values.rpm);
	DEBUGSERIAL.print("v_in: 				"); DEBUGSERIAL.println(values.v_in);
	DEBUGSERIAL.print("duty_now: 			"); DEBUGSERIAL.println(values.duty_now);
	DEBUGSERIAL.print("amp_hours: 			"); DEBUGSERIAL.println(values.amp_hours);
	DEBUGSERIAL.print("amp_hours_charged: 	"); DEBUGSERIAL.println(values.amp_hours_charged);
	DEBUGSERIAL.print("watt_hours: 			"); DEBUGSERIAL.println(values.watt_hours);
	DEBUGSERIAL.print("watt_hours_charged: 	"); DEBUGSERIAL.println(values.watt_hours_charged);
	DEBUGSERIAL.print("tachometer: 			"); DEBUGSERIAL.println(values.tachometer);
	DEBUGSERIAL.print("tachometer_abs: 		"); DEBUGSERIAL.println(values.tachometer_abs);
	DEBUGSERIAL.print("position:	 		"); DEBUGSERIAL.println(values.position);
	DEBUGSERIAL.print("fault_code: 			"); DEBUGSERIAL.println(bldc_interface_fault_to_string(values.fault_code));

}