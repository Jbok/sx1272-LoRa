#include "mbed.h"
#include "stdint.h"
#include "sx1272/sx1272Regs-Fsk.h"
#include "drivers/Ticker.h"

/* Set this flag to '1' to use the LoRa modulation or to '0' to use FSK modulation */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   !USE_MODEM_LORA

#define RF_FREQUENCY                                    868000000 // Hz
#define TX_OUTPUT_POWER                                 14        // 14 dBm

#if USE_MODEM_LORA == 1

#define LORA_BANDWIDTH                              2			// [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7			  // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_FHSS_ENABLED                           false  
#define LORA_NB_SYMB_HOP                            4     
#define LORA_IQ_INVERSION_ON                        false
#define LORA_CRC_ENABLED                            true

#elif USE_MODEM_FSK == 1

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                19200     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#define FSK_CRC_ENABLED                             true

#else
#error "Please define a modem in the compiler options."
#endif

#define RX_TIMEOUT_VALUE                                3500      // in ms
#define BUFFER_SIZE                                     256        // Define the payload size here

/*
*  Global variables declarations
*/
typedef enum
{
	LOWPOWER = 0,
	IDLE,

	RX,
	RX_TIMEOUT,
	RX_ERROR,

	TX,
	TX_TIMEOUT,

	CAD,
	CAD_DONE
}AppStates_t;

volatile AppStates_t State = LOWPOWER;

typedef enum RadioState
{
	RF_IDLE = 0,
	RF_RX_RUNNING,
	RF_TX_RUNNING,
	RF_CAD,
}RadioState_t;

typedef enum ModemType {
	MODEM_FSK = 0,
	MODEM_LORA
}RadioModems_t;

typedef struct {
	bool isReceived = false;
	bool rxContinuous = true;
	bool fskRxContinuous = true;
	bool freqHopOn = false;
	RadioState_t state;
	AppStates_t appState;
	RadioModems_t modem;
	uint32_t TxTimeout = 2000;
	int8_t snrValue;
	int16_t rssiValue;
	uint8_t sizeValue;
	uint8_t fifoThresh;
	bool preambleDetected;
	bool syncWordDetected;
	uint16_t nbBytes;
	uint16_t fskSize;
	bool fskCrcOn;
	bool fskFixLen;
	uint32_t fskRxSingleTimeout = 0;
	uint8_t chunkSize;
}sx1272;



int16_t RssiValue = 0.0;
int8_t SnrValue = 0.0;


uint8_t PingMsg[] = "Orci varius natoque penatibus et magnis dis parturient montes, nascetur ridiculus mus. Mauris dolor ligula, dapibus dictum orci eu, feugiat lobortis purus. Suspendisse potenti. Nullam id rutrum nulla, vestibulum porttitor tortor. Fusce lacinia cursus sed.";
//const uint8_t PingMsg[] = "ACK";
uint8_t len_pingmsg = (uint8_t)strlen((const char*)PingMsg);


uint16_t BufferSize = len_pingmsg;
uint8_t Buffer[BUFFER_SIZE];

//Frequecny Synthesis
#define TS_OSC			250		//us	Crystal oscillator wake-up time
#define TS_FS			60		//us	Frequeyncy synthesizer wake-up time to PllLock Signal
#define TS_TR			120		//us	Transmitter wake up time, to the first risging edge of DCLK


//Reg Op Mode
#define LongRangeMode	0x80
#define AccesShredReg	0x40
//Device Mode
#define RF_OPMODE_MASK                              0xF8
#define SLEEP			0x00
#define STDBY			0x01
#define FSTX			0x02
#define RFLR_OPMODE_TRANSMITTER                     0x03 
#define FSRX			0x04
#define RXCONTINUOUS	0x05
#define RXSINGLE		0x06
#define CAD				0x07



//LORA Common Register Settings
#define REG_OP_MODE									0x01
#define REG_FR_MSB									0x06
#define REG_FR_MIB									0x07
#define REG_FR_LSB									0x08

// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0D 
#define REG_LR_FIFOTXBASEADDR                       0x0E 
#define REG_LR_FIFORXBASEADDR                       0x0F 
#define REG_LR_FIFORXCURRENTADDR                    0x10 
#define REG_LR_IRQFLAGSMASK                         0x11 
#define REG_LR_IRQFLAGS                             0x12 
#define REG_LR_RXNBBYTES                            0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                  0x14 
#define REG_LR_RXHEADERCNTVALUELSB                  0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                  0x16 
#define REG_LR_RXPACKETCNTVALUELSB                  0x17 
#define REG_LR_MODEMSTAT                            0x18 
#define REG_LR_PKTSNRVALUE                          0x19 
#define REG_LR_PKTRSSIVALUE                         0x1A 
#define REG_LR_RSSIVALUE                            0x1B 
#define REG_LR_HOPCHANNEL                           0x1C 
#define REG_LR_MODEMCONFIG1                         0x1D 
#define REG_LR_MODEMCONFIG2                         0x1E 
#define REG_LR_SYMBTIMEOUTLSB                       0x1F 
#define REG_LR_PREAMBLEMSB                          0x20 
#define REG_LR_PREAMBLELSB                          0x21 
#define REG_LR_PAYLOADLENGTH                        0x22 
#define REG_LR_PAYLOADMAXLENGTH                     0x23 
#define REG_LR_HOPPERIOD                            0x24 
#define REG_LR_FIFORXBYTEADDR                       0x25
#define REG_LR_FEIMSB                               0x28
#define REG_LR_FEIMID                               0x29
#define REG_LR_FEILSB                               0x2A
#define REG_LR_RSSIWIDEBAND                         0x2C
#define REG_LR_DETECTOPTIMIZE                       0x31
#define REG_LR_INVERTIQ                             0x33
#define REG_LR_DETECTIONTHRESHOLD                   0x37
#define REG_LR_SYNCWORD                             0x39
#define REG_LR_INVERTIQ2                            0x3B

// I/O Settings
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41

//Version register
#define REG_VERSION									0x42


/*!
* RegIrqFlagsMask
*/
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80 
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20 
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10 
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08 
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02 
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01 
/*!
* RegIrqFlags
*/
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80 
#define RFLR_IRQFLAGS_RXDONE                        0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20 
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10 
#define RFLR_IRQFLAGS_TXDONE                        0x08 
#define RFLR_IRQFLAGS_CADDONE                       0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02 
#define RFLR_IRQFLAGS_CADDETECTED                   0x01 

/*!
* RegDioMapping1
*/
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C




//Frequency Settings
#define XTAL_FREQ				32e6
#define FREQ_STEP				61.035

#define RX_BUFFER_SIZE			256

/*!
* Constant values need to compute the RSSI value
*/
#define RSSI_OFFSET                                 -139



SPI spi(PA_7, PA_6, PA_5);				//MOSI, MISO, SCK
DigitalOut nss(PA_4);					//NSS
DigitalInOut reset(PB_2);				//RESET
										//DigitalInOut antena(PC_4);				//Antena Switch: high in Tx

InterruptIn dio0(PC_5);
InterruptIn dio1(PB_0);
InterruptIn dio2(PB_1);



uint8_t rxtxBuffer[BUFFER_SIZE];

sx1272 radio;

uint8_t read_reg(uint8_t addr) {
	uint8_t value;
	nss = 0;
	spi.write(0x7F & addr);
	value = spi.write(0x00);
	nss = 1;
	return value;
}

void write_reg(uint8_t addr, uint8_t value) {
	nss = 0;
	spi.write(0x80 | addr);
	spi.write(value);
	nss = 1;
}

void write_fifo(uint8_t *buffer, uint8_t size) {
	nss = 0;
	spi.write(0x80);
	for (uint8_t i = 0; i < size; i++) {
		spi.write(buffer[i]);
	}
	nss = 1;
}

void read_fifo(uint8_t *buffer, uint8_t size) {
	nss = 0;
	spi.write(0x00);
	for (uint8_t i = 0; i < size; i++) {
		buffer[i] = spi.write(0x00);
	}
	nss = 1;
}

void set_channel(uint32_t freq) {
	freq = (uint32_t)((double)freq / (double)FREQ_STEP);
	write_reg(REG_FR_MSB, (uint8_t)((freq >> 16) & 0xFF));
	write_reg(REG_FR_MIB, (uint8_t)((freq >> 8) & 0xFF));
	write_reg(REG_FR_LSB, (uint8_t)(freq & 0xFF));
}

void set_modem(RadioModems_t modem) {
	switch (modem) {
	case MODEM_LORA:
		//sleep
		write_reg(REG_OP_MODE, SLEEP);

		write_reg(REG_DIOMAPPING1, 0x00);
		write_reg(REG_DIOMAPPING2, 0x00);
		break;
	}
}


void reset_radio() {
	reset.output();
	reset = 0;
	wait_ms(1);
	reset.input();
	wait_ms(6);
}


void on_dio0_irq() {
	uint8_t irqFlags = 0;

	switch (radio.state)
	{
	case RF_RX_RUNNING:
		switch (radio.modem)
		{
		case MODEM_LORA:
			int8_t snr = 0;

			//Clear Irq
			write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);


			irqFlags = read_reg(REG_LR_IRQFLAGS);
			if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
			{
				// Clear Irq
				write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

				if (radio.rxContinuous == false)
				{
					radio.state = RF_IDLE;
				}

				/*RxError*/
				write_reg(REG_OP_MODE, SLEEP);
				radio.appState = RX_ERROR;
				printf("> OnRxError\n\r");
				break;
			}

			//SNR
			//The SNR sign bit is 1
			//	Invert and divide by 4	SNR[db] = PacketSnr[Twos complement]/4
			radio.snrValue = read_reg(REG_LR_PKTSNRVALUE);
			if (radio.snrValue & 0x80) {
				snr = (((~radio.snrValue + 1)) & 0xFF) >> 2;
				snr = -snr;
			}
			else {
				//	Divide by 4
				snr = (((radio.snrValue + 1)) & 0xFF) >> 2;
			}

			//RSSI
			/*
			RSSI[dBm] = -139 + PacketRssi (SNR>=0)
			-139 + packetRssi + PacketSnr*0.25 (SNR<0)
			*/
			int16_t rssi = read_reg(REG_LR_PKTRSSIVALUE);
			if (snr < 0) {
				radio.rssiValue = RSSI_OFFSET + rssi + (rssi >> 4) + snr;
			}
			else {
				radio.rssiValue = RSSI_OFFSET + rssi + (rssi >> 4);
			}

			//LORA
			//Number of payload bytes of latest packet received

			radio.sizeValue = read_reg(REG_LR_RXNBBYTES);
			rxtxBuffer[0] = '\0';
			read_fifo(rxtxBuffer, radio.sizeValue);
			if (radio.rxContinuous == false) {
				radio.state = RF_IDLE;
			}


			//RxDone
			write_reg(REG_OP_MODE, SLEEP);
			BufferSize = radio.sizeValue;
			Buffer[0] = '\0';	//remove Buffer
			memcpy(Buffer, rxtxBuffer, BufferSize);
			Buffer[BufferSize] = '\0';
			wait_ms(1);
			RssiValue = rssi;
			SnrValue = snr;
			radio.appState = RX;
			printf("> OnRxDone\n\r");
			break;

		}
		break;
	case RF_TX_RUNNING:
		switch (radio.modem)
		{
		case MODEM_LORA:
			//Clear Irq
			write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
		default:
			radio.state = RF_IDLE;
			/*TxDone*/
			write_reg(REG_OP_MODE, SLEEP);
			radio.appState = TX;
			printf("> OnTxDone\n\r");
			wait_us(10);
			break;
		}

	}
}

void on_dio1_irq() {
	printf("Interrupt 1\r\n");
	switch (radio.state) {
	case RF_RX_RUNNING:
		switch (radio.modem) {

		case MODEM_LORA:


			//	Clear IRQ
			write_reg(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

			radio.state = RF_IDLE;
			/*RxTimeOut*/
			//sleep
			write_reg(REG_OP_MODE, SLEEP);

			Buffer[BufferSize] = 0;
			break;
		default:
			break;
		}
	case RF_TX_RUNNING:
		switch (radio.modem) {
		case MODEM_LORA:
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

void on_dio2_irq() {
	printf("Interrupt 2\r\n");
	switch (radio.appState)
	{
	default:
		break;
	}
}

void IoIrqInit() {
	dio0.mode(PullDown);
	dio1.mode(PullDown);
	dio2.mode(PullDown);

	printf("dio0 %d, dio1 %d, dio2 %d\r\n", dio0.read(), dio1.read(), dio2.read());


	dio0.rise(&on_dio0_irq);
	dio1.rise(&on_dio1_irq);
	dio2.rise(&on_dio2_irq);
}

void rx() {
	bool rxContinuous = false;

	switch (radio.modem) {
	case MODEM_LORA:
		/*IqInverted*/
		//
		//		Not Implemented
		//

		rxContinuous = radio.rxContinuous;


		/*FreqHopOn*/
		/*MODEM_LORA*/
		if (radio.freqHopOn == true) {
			write_reg(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_CADDETECTED);

			// DIO0=RxDone, DIO2=FhssChangeChannel
			write_reg(REG_DIOMAPPING1, (read_reg(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
		}
		else {
			write_reg(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_VALIDHEADER |
				RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

			// DIO0=RxDone
			write_reg(REG_DIOMAPPING1, (read_reg(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
		}
		write_reg(REG_LR_FIFORXBASEADDR, 0x00);
		write_reg(REG_LR_FIFOADDRPTR, 0x00);
		break;
	}


	//set Buffer
	//memset(rxtxBuffer, 0, (size_t)RX_BUFFER_SIZE);

	radio.state = RF_RX_RUNNING;




	//MODEM_LORA
	if (rxContinuous == true) {
		write_reg(REG_OP_MODE, (read_reg(REG_OP_MODE) & RF_OPMODE_MASK) | RXCONTINUOUS);
	}
	else {
		write_reg(REG_OP_MODE, (read_reg(REG_OP_MODE) & RF_OPMODE_MASK) | RXSINGLE);
	}


}

void tx() {
	switch (radio.modem) {
	case MODEM_LORA:
		if (radio.freqHopOn == true) {
			write_reg(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
				RFLR_IRQFLAGS_RXDONE |
				RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				//RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				//RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

			// DIO0=TxDone, DIO2=FhssChangeChannel
			write_reg(REG_DIOMAPPING1, (read_reg(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
		}
		else {
			write_reg(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
				RFLR_IRQFLAGS_RXDONE |
				RFLR_IRQFLAGS_PAYLOADCRCERROR |
				RFLR_IRQFLAGS_VALIDHEADER |
				//RFLR_IRQFLAGS_TXDONE |
				RFLR_IRQFLAGS_CADDONE |
				RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
				RFLR_IRQFLAGS_CADDETECTED);

			// DIO0=TxDone
			write_reg(REG_DIOMAPPING1, (read_reg(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
		}
		break;
	}

	radio.state = RF_TX_RUNNING;
	write_reg(REG_OP_MODE, (read_reg(REG_OP_MODE)&RF_OPMODE_MASK) | RFLR_OPMODE_TRANSMITTER);
}

void send(uint8_t *buffer, uint8_t size) {

	switch (radio.modem)
	{
	case MODEM_FSK:
		break;
	case MODEM_LORA:
		// Initializes the payload size
		write_reg(REG_LR_PAYLOADMAXLENGTH, 0xFF);
		write_reg(REG_LR_PAYLOADLENGTH, size);

		// Full buffer used for Tx
		write_reg(REG_LR_FIFOTXBASEADDR, 0);
		write_reg(REG_LR_FIFOADDRPTR, 0);

		// Change mode If Sleep mode
		if ((read_reg(REG_OP_MODE) & 0x07) == 0x00) {
			//Set standby
			write_reg(REG_OP_MODE, STDBY);
			radio.state = RF_IDLE;

			wait_ms(1);
		}

		//Write payload buffer
		printf("Send Size : %d \r\n", size);
		write_fifo(buffer, size);
		break;
	}
	tx();

}

void reset_reg_lr_irq_flags() {
	write_reg(REG_LR_IRQFLAGS, 0x00);
}


void set_lora_mode() {
	//sleep
	write_reg(REG_OP_MODE, SLEEP);
	write_reg(REG_OPMODE, RF_OPMODE_LONGRANGEMODE_ON);
	write_reg(REG_LR_MODEMCONFIG1, 0b01001010);
	write_reg(REG_LR_MODEMCONFIG2, 0b01110100);
}


#define TRANSMODE 1
#define RECEIVEMODE 0

int main() {
	int mode = RECEIVEMODE;

	printf("BufferSize :%d\r\n", BufferSize);
	set_lora_mode();


	if (USE_MODEM_LORA == 1) {
		radio.modem = MODEM_LORA;
		reset_reg_lr_irq_flags();
	}
	else {
		radio.modem = MODEM_FSK;
	}


	IoIrqInit();

	//verify the connection with the board.
	while (read_reg(REG_VERSION) == 0x00) {
		printf("Radio could not be detected!\n\r");
		wait(1);
	}

	bool isMaster = true;

	set_channel(RF_FREQUENCY);

	//set configure
	printf("\n\n\n\rLet's Starat!\r\n");


	if (mode == TRANSMODE) {
		printf("=======Trasmit Device=======\r\n\r\n\r\n\r\n");
		strcpy((char*)Buffer, (char*)PingMsg);

		//BufferSize = BUFFER_SIZE;
		for (uint8_t i = len_pingmsg; i < BufferSize - 1; i++) {
			Buffer[i] = i - len_pingmsg;
		}
		wait_ms(10);
		send(Buffer, len_pingmsg);
	}
	else {
		printf("=======Receive Device=======\r\n\r\n\r\n\r\n");
		rx();
	}


	while (1) {
		wait_ms(1);
		switch (radio.appState) {
		case RX:
			if (isMaster == true) {
				if (BufferSize > 0) {


					if (!strcmp((char*)Buffer, "ack")) {
						printf("Received Message: %s\r\n", Buffer);
						radio.appState = LOWPOWER;
						break;
					}
					else {
						printf("Received Message: %s\r\n", Buffer);
						strcpy((char*)PingMsg, (char*)"ack");
					}

					strcpy((char*)Buffer, (char*)PingMsg);

					for (uint8_t i = len_pingmsg; i < BufferSize - 1; i++) {
						Buffer[i] = i - len_pingmsg;
					}
					wait_ms(10);
					send(Buffer, len_pingmsg);
				}
			}
			radio.appState = LOWPOWER;
			break;
		case TX:
			if (isMaster == true) {
				printf("Send Message [%s]\r\n", PingMsg);
			}
			else {
				printf("Receive Message [%s]\r\n", Buffer);
			}
			rx();
			radio.appState = LOWPOWER;
			break;
		case RX_ERROR:
			//	Send Packet 
			rx();

			radio.appState = LOWPOWER;
			break;
		case LOWPOWER:
			break;
		default:
			radio.appState = LOWPOWER;
			break;
		}

	}
}