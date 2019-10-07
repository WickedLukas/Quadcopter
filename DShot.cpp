#include "DSHOT.h"

DShot::DShot(uint8_t num) {
	setup (num);
}

void DShot::setup(uint8_t num) {
	// FTM0 configuration
	FTM0_SC = 0;		// disable FTM0 for initialization
	FTM0_CNT = 0;		// reset counter
	FTM0_CNTIN = 0;		// reset counter
	FTM0_MOD = mod - 1;	// set modulo (overflow) that triggers the timer event
	FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); 	// set clock source to 1 (system clock) and prescale to 0 (division by 1)
	
	// channel configuration
	if (num == 1) {
		FTM0_CnSC(OUT1_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
		FTM0_CnSC(OUT1_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
		FTM0_CnV(OUT1_TRIG_CH)  = 0;
		FTM0_CnV(OUT1_PWM_CH)   = 0;
		
		dma.destination(FTM0_CnV(OUT1_PWM_CH));	// feed PWM value to output channel
		dma.triggerAtHardwareEvent(DMA_SOURCE_FTM0_CHn(OUT1_TRIG_CH));	// set DMA transfer source (trigger channel)
		
		// configure PWM output pin
		FTM_PINCFG(OUT1_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
	else if (num == 2) {
		FTM0_CnSC(OUT2_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
		FTM0_CnSC(OUT2_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
		FTM0_CnV(OUT2_TRIG_CH)  = 0;
		FTM0_CnV(OUT2_PWM_CH)   = 0;
		
		dma.destination(FTM0_CnV(OUT2_PWM_CH));	// feed PWM value to output channel
		dma.triggerAtHardwareEvent(DMA_SOURCE_FTM0_CHn(OUT2_TRIG_CH));	// set DMA transfer source (trigger channel)
		
		// configure PWM output pin
		FTM_PINCFG(OUT2_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
	else if (num == 3) {
		FTM0_CnSC(OUT3_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
		FTM0_CnSC(OUT3_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
		FTM0_CnV(OUT3_TRIG_CH)  = 0;
		FTM0_CnV(OUT3_PWM_CH)   = 0;
		
		dma.destination(FTM0_CnV(OUT3_PWM_CH));	// feed PWM value to output channel
		dma.triggerAtHardwareEvent(DMA_SOURCE_FTM0_CHn(OUT3_TRIG_CH));	// set DMA transfer source (trigger channel)
		
		// configure PWM output pin
		FTM_PINCFG(OUT3_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
	else if (num == 4) {
		FTM0_CnSC(OUT4_TRIG_CH) = FTM_CSC_CHIE | FTM_CSC_DMA | FTM_CSC_MSA | FTM_CSC_ELSA;
		FTM0_CnSC(OUT4_PWM_CH)  = FTM_CSC_MSB | FTM_CSC_ELSB;
		FTM0_CnV(OUT4_TRIG_CH)  = 0;
		FTM0_CnV(OUT4_PWM_CH)   = 0;
		
		dma.destination(FTM0_CnV(OUT4_PWM_CH));	// feed PWM value to output channel
		dma.triggerAtHardwareEvent(DMA_SOURCE_FTM0_CHn(OUT4_TRIG_CH));	// set DMA transfer source (trigger channel)
		
		// configure PWM output pin
		FTM_PINCFG(OUT4_PWM_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
	}
	
	// pin-independent DMA configuration
	dma.sourceBuffer((uint8_t *)dshotBuffer, DSHOT_BUFFER_LENGTH);	// source buffer
	dma.transferSize(1);
	dma.transferCount(DSHOT_BUFFER_LENGTH);
	dma.disableOnCompletion();
}

uint8_t DShot::getChecksum(uint16_t value) {
	uint8_t checksum = 0;
	
	for (uint8_t i = 0; i < 3; i++) {
		checksum ^=  value;
		value >>= 4;
	}
	
	checksum &= 0xf;
	
	return checksum;
}

void DShot::fillBuffer(uint16_t value) {
	memset(dshotBuffer, 0, DSHOT_BUFFER_LENGTH);
	
	// scan all bits inside the packet
	for (uint8_t i = 0; i < DSHOT_COMMAND_LENGTH; i++) {
		if ((bool) ((1 << i) & value)) {
			dshotBuffer[15 - i] = (mod * DSHOT_1_TIMING) >> 8;	// fill buffer, MSB first
		}
		else {
			dshotBuffer[15 - i] = (mod * DSHOT_0_TIMING) >> 8;	// fill buffer, MSB first
		}
	}
}

void DShot::setThrottle(uint16_t value) {
	write(value + 47);
}


void DShot::setDirection(bool dir) {
	uint8_t value = 0;
	
	if (dir) {
		value = 7;
	}
	else {
		value = 8;
	}
	
	// send setting 10 times
	for(uint8_t i = 0; i < 10; i++) {
		requestTelemetry();
		write(value);
		delayMicroseconds(500);	
	}
	
	delay(100);
	
	// save settings
	for (uint8_t i = 0; i < 10; i++) {
		requestTelemetry();
		write(12);
		delayMicroseconds(500);
	}
}

void DShot::setNormal() {
	uint8_t value = 9;
	
	// send setting 10 times
	for (uint8_t i = 0; i < 10; i++) {
		requestTelemetry();
		write( value );
		delayMicroseconds(500);
	}
	
	delay(100);
	
	// save settings
	for( uint8_t i = 0; i < 10; i++){
		requestTelemetry();
		write( 12 );
		delayMicroseconds(500);
	}
}

void DShot::write(uint16_t value) {
	uint16_t packet = 0;
	uint8_t checksum = 0;
	
	if (value > 2047) {
		value = 2047;
	}
	
	// append telemetry bit
	packet = (value << 1) | (uint8_t) requestTlm;
	requestTlm = false;
	
	checksum = getChecksum(packet);
	packet = (packet << 4) | checksum;
	
	fillBuffer( packet );
	
	// write package by enabling DMA
	dma.enable();
}

void DShot::requestTelemetry() {
	requestTlm = true;
}

bool DShot::readTelemetry(Stream *tlmPort) {
	bool tlmDone = false;
	uint8_t buf[10] = {'\0'};
	uint8_t num = 0;
	uint32_t lastTime = 0;

	if (tlmPort->available() > 0) {
		lastTime = micros();
		
		while (tlmDone == false && (micros() - lastTime) < 500) {
			if (tlmPort->available()) {
				buf[num++] = tlmPort->read();
				lastTime = micros();
			}

			if (num == 10) {
				tlmDone = true;	
			}
		}
	}
	
	if (tlmDone == true) {
		// clear serial buffer for future readings
		while (tlmPort->available() > 0) {
			tlmPort->read();
		}
		
		// calculate 8bit CRC to validate telemetry
		uint8_t valid = get_crc8(buf, 9);
		
		if (valid == buf[9]) {
			// telemetry received successfully (CRC valid)
			tlm.temp       	= (float) (buf[0]);
			tlm.voltage    	= (float) ((buf[1]<<8)|buf[2]) / 100.0;
			tlm.amps       	= (float) ((buf[3]<<8)|buf[4]) / 100.0;
			tlm.ampHours   	= (float) ((buf[5]<<8)|buf[6]);
			tlm.rpm        	= (float) ((buf[7]<<8)|buf[8]) * 100.0 / 7.0;
			tlm.timestamp 	= micros();
			return true;
		}
	}
	return false;
}

uint8_t DShot::update_crc8(uint8_t crc, uint8_t crcSeed) {
	uint8_t crc_u;
	
	crc_u = crc;
	crc_u ^= crcSeed;
	
	for ( uint8_t i = 0; i < 8; i++ ) {
		crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
	}
	
	return crc_u;
}

uint8_t DShot::get_crc8(uint8_t *buf, uint8_t bufLen) {
	uint8_t crc = 0;
	
	for (uint8_t i = 0; i < bufLen; i++) {
		crc = update_crc8(buf[i], crc);
	}
	
	return crc;
}