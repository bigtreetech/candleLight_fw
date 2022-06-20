/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can.h"
#include "config.h"

#if defined(STM32G0)
#define CAN_TypeDef  FDCAN_GlobalTypeDef

FDCAN_HandleTypeDef hfdcan2;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t	rx_can2_ok = 0;
can_data_t *_hcan;

uint8_t TxData[64]={0};
uint8_t RxData[64]={0};

#define CAN_ESR_EWGF_Pos       (0U)                                            
#define CAN_ESR_EWGF_Msk       (0x1UL << CAN_ESR_EWGF_Pos)                      /*!< 0x00000001 */
#define CAN_ESR_EWGF           CAN_ESR_EWGF_Msk                                /*!<Error Warning Flag */
#define CAN_ESR_EPVF_Pos       (1U)                                            
#define CAN_ESR_EPVF_Msk       (0x1UL << CAN_ESR_EPVF_Pos)                      /*!< 0x00000002 */
#define CAN_ESR_EPVF           CAN_ESR_EPVF_Msk                                /*!<Error Passive Flag */
#define CAN_ESR_BOFF_Pos       (2U)                                            
#define CAN_ESR_BOFF_Msk       (0x1UL << CAN_ESR_BOFF_Pos)                      /*!< 0x00000004 */
#define CAN_ESR_BOFF           CAN_ESR_BOFF_Msk                                /*!<Bus-Off Flag */

#define CAN_ESR_LEC_Pos        (4U)                                            
#define CAN_ESR_LEC_Msk        (0x7UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000070 */
#define CAN_ESR_LEC            CAN_ESR_LEC_Msk                                 /*!<LEC[2:0] bits (Last Error Code) */
#define CAN_ESR_LEC_0          (0x1UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000010 */
#define CAN_ESR_LEC_1          (0x2UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000020 */
#define CAN_ESR_LEC_2          (0x4UL << CAN_ESR_LEC_Pos)                       /*!< 0x00000040 */

#define CAN_ESR_TEC_Pos        (16U)                                           
#define CAN_ESR_TEC_Msk        (0xFFUL << CAN_ESR_TEC_Pos)                      /*!< 0x00FF0000 */
#define CAN_ESR_TEC            CAN_ESR_TEC_Msk                                 /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_REC_Pos        (24U)                                           
#define CAN_ESR_REC_Msk        (0xFFUL << CAN_ESR_REC_Pos)                      /*!< 0xFF000000 */
#define CAN_ESR_REC            CAN_ESR_REC_Msk                                 /*!<Receive Error Counter */

#endif

void can_init(can_data_t *hcan, CAN_TypeDef *instance)
{

#if defined(STM32F0) ||	defined(STM32F4)
	__HAL_RCC_CAN1_CLK_ENABLE();

	GPIO_InitTypeDef itd;
#endif

#if defined(STM32F0)
	itd.Pin = GPIO_PIN_8|GPIO_PIN_9;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_HIGH;
	itd.Alternate = GPIO_AF4_CAN;
	HAL_GPIO_Init(GPIOB, &itd);
#elif defined(STM32F4)
	itd.Pin = GPIO_PIN_0|GPIO_PIN_1;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	itd.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &itd);
#endif

#if defined(STM32F0) || defined(STM32F4)
	hcan->instance   = instance;
#elif defined(STM32G0)
	hcan->instance   = instance;
	hcan->instance   = FDCAN2;
#endif

	hcan->brp        = 6;
	hcan->sjw		 = 1;
#if defined(STM32F0) || defined(STM32G0)
	hcan->phase_seg1 = 13;
	hcan->phase_seg2 = 2;
#elif defined(STM32F4)
	hcan->phase_seg1 = 12;
	hcan->phase_seg2 = 1;
#endif
	
}

bool can_set_bittiming(can_data_t *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if ( (brp>0) && (brp<=1024)
	  && (phase_seg1>0) && (phase_seg1<=16)
	  && (phase_seg2>0) && (phase_seg2<=8)
	  && (sjw>0) && (sjw<=4)
	) {
		hcan->brp = brp & 0x3FF;
		hcan->phase_seg1 = phase_seg1;
		hcan->phase_seg2 = phase_seg2;
		hcan->sjw = sjw;
		return true;
	} else {
		return false;
	}
}

#if defined(STM32G0)
void FDCAN_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;
	sFilterConfig.FilterID2 = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
#endif

void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot)
{

#if defined(STM32F0) || defined(STM32F4)

	CAN_TypeDef *can = hcan->instance;

	uint32_t mcr = CAN_MCR_INRQ
				 | CAN_MCR_ABOM
				 | CAN_MCR_TXFP
				 | (one_shot ? CAN_MCR_NART : 0);

	uint32_t btr = ((uint32_t)(hcan->sjw-1)) << 24
				 | ((uint32_t)(hcan->phase_seg1-1)) << 16
				 | ((uint32_t)(hcan->phase_seg2-1)) << 20
				 | (hcan->brp - 1)
				 | (loop_back ? CAN_MODE_LOOPBACK : 0)
				 | (listen_only ? CAN_MODE_SILENT : 0);


	// Reset CAN peripheral
	can->MCR |= CAN_MCR_RESET;
	while((can->MCR & CAN_MCR_RESET) != 0); // reset bit is set to zero after reset
	while((can->MSR & CAN_MSR_SLAK) == 0);  // should be in sleep mode after reset

	can->MCR |= CAN_MCR_INRQ ;
	while((can->MSR & CAN_MSR_INAK) == 0);

	can->MCR = mcr;
	can->BTR = btr;

	can->MCR &= ~CAN_MCR_INRQ;
	while((can->MSR & CAN_MSR_INAK) != 0);

	uint32_t filter_bit = 0x00000001;
	can->FMR |= CAN_FMR_FINIT;
	can->FMR &= ~CAN_FMR_CAN2SB;
	can->FA1R &= ~filter_bit;        // disable filter
	can->FS1R |= filter_bit;         // set to single 32-bit filter mode
	can->FM1R &= ~filter_bit;        // set filter mask mode for filter 0
	can->sFilterRegister[0].FR1 = 0; // filter ID = 0
	can->sFilterRegister[0].FR2 = 0; // filter Mask = 0
	can->FFA1R &= ~filter_bit;       // assign filter 0 to FIFO 0
	can->FA1R |= filter_bit;         // enable filter
	can->FMR &= ~CAN_FMR_FINIT;

#elif defined(STM32G0)
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;

	if(listen_only)
	{
		hfdcan2.Init.Mode = FDCAN_MODE_BUS_MONITORING;	
	}
	else
	{
		if(loop_back)
		{
			hfdcan2.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;	
		}
		else
		{
			hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;	
		}
	}	

	if(one_shot)
	{
	  hfdcan2.Init.AutoRetransmission = ENABLE;
	}
	else
	{
	  hfdcan2.Init.AutoRetransmission = DISABLE;
	}

	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;

	hfdcan2.Init.NominalPrescaler = hcan->brp;
	hfdcan2.Init.NominalSyncJumpWidth = hcan->sjw;
	hfdcan2.Init.NominalTimeSeg1 = hcan->phase_seg1;
	hfdcan2.Init.NominalTimeSeg2 = hcan->phase_seg2;
		
	hfdcan2.Init.DataPrescaler = hcan->brp;
	hfdcan2.Init.DataSyncJumpWidth = hcan->sjw;
	hfdcan2.Init.DataTimeSeg1 = hcan->phase_seg1;
	hfdcan2.Init.DataTimeSeg2 = hcan->phase_seg2;

	hfdcan2.Init.StdFiltersNbr = 1;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	
    HAL_FDCAN_Init(&hfdcan2);
	FDCAN_Config();

#endif

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_SET);
#endif
}

void can_disable(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_RESET);
#endif

#if defined(STM32F0) || defined(STM32F4)
	can->MCR |= CAN_MCR_INRQ ; // send can controller into initialization mode
#elif defined(STM32G0)
	can->CCCR |= FDCAN_CCCR_INIT ; // send can controller into initialization mode
#endif

}



bool can_is_enabled(can_data_t *hcan)
{
CAN_TypeDef *can = hcan->instance;
#if defined(STM32F0) || defined(STM32F4)
	return (can->MCR & CAN_MCR_INRQ) == 0;
#elif defined(STM32G0)
	return (can->CCCR & FDCAN_CCCR_INIT) == 0;
#endif
}



bool can_is_rx_pending(can_data_t *hcan)
{
#if defined(STM32F0) || defined(STM32F4)	
	CAN_TypeDef *can = hcan->instance;
	return ((can->RF0R & CAN_RF0R_FMP0) != 0);
#elif defined(STM32G0)
    _hcan = hcan;
	return (rx_can2_ok != 0);
#endif	
}

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame)
{

#if defined(STM32F0) || defined(STM32F4)

	CAN_TypeDef *can = hcan->instance;

	if (can_is_rx_pending(hcan)) {
		CAN_FIFOMailBox_TypeDef *fifo = &can->sFIFOMailBox[0];

		if (fifo->RIR &  CAN_RI0R_IDE) {
			rx_frame->can_id = CAN_EFF_FLAG | ((fifo->RIR >> 3) & 0x1FFFFFFF);
		} else {
			rx_frame->can_id = (fifo->RIR >> 21) & 0x7FF;
		}

		if (fifo->RIR & CAN_RI0R_RTR)  {
			rx_frame->can_id |= CAN_RTR_FLAG;
		}

		rx_frame->can_dlc = fifo->RDTR & CAN_RDT0R_DLC;

		rx_frame->data[0] = (fifo->RDLR >>  0) & 0xFF;
		rx_frame->data[1] = (fifo->RDLR >>  8) & 0xFF;
		rx_frame->data[2] = (fifo->RDLR >> 16) & 0xFF;
		rx_frame->data[3] = (fifo->RDLR >> 24) & 0xFF;
		rx_frame->data[4] = (fifo->RDHR >>  0) & 0xFF;
		rx_frame->data[5] = (fifo->RDHR >>  8) & 0xFF;
		rx_frame->data[6] = (fifo->RDHR >> 16) & 0xFF;
		rx_frame->data[7] = (fifo->RDHR >> 24) & 0xFF;

		can->RF0R |= CAN_RF0R_RFOM0; // release FIFO

		return true;
	} else {
		return false;
	}
#elif defined(STM32G0)
uint16_t  i=0;
	
	_hcan = hcan;

	rx_frame->can_id = RxHeader.Identifier;
	rx_frame->can_dlc = (uint8_t)(RxHeader.DataLength>>16);

	rx_frame->data[0] = RxData[0];
	rx_frame->data[1] = RxData[1];
	rx_frame->data[2] = RxData[2];
	rx_frame->data[3] = RxData[3];
	rx_frame->data[4] = RxData[4];
	rx_frame->data[5] = RxData[5];
	rx_frame->data[6] = RxData[6];
	rx_frame->data[7] = RxData[7];	

	for(i=0;i<8;i++)
	{
		RxData[i]=0;
	}
	return true;
#endif
}

#if defined(STM32F0) || defined(STM32F4)
static CAN_TxMailBox_TypeDef *can_find_free_mailbox(can_data_t *hcan)
{
	CAN_TypeDef *can = hcan->instance;

	uint32_t tsr = can->TSR;
	if ( tsr & CAN_TSR_TME0 ) {
		return &can->sTxMailBox[0];
	} else if ( tsr & CAN_TSR_TME1 ) {
		return &can->sTxMailBox[1];
	} else if ( tsr & CAN_TSR_TME2 ) {
		return &can->sTxMailBox[2];
	} else {
		return 0;
	}
}
#endif

bool can_send(can_data_t *hcan, struct gs_host_frame *frame)
{
#if defined(STM32F0) || defined(STM32F4)

	CAN_TxMailBox_TypeDef *mb = can_find_free_mailbox(hcan);
	if (mb != 0) {

		/* first, clear transmission request */
		mb->TIR &= CAN_TI0R_TXRQ;

		if (frame->can_id & CAN_EFF_FLAG) { // extended id
			mb->TIR = CAN_ID_EXT | (frame->can_id & 0x1FFFFFFF) << 3;
		} else {
			mb->TIR = (frame->can_id & 0x7FF) << 21;
		}

		if (frame->can_id & CAN_RTR_FLAG) {
			mb->TIR |= CAN_RTR_REMOTE;
		}

		mb->TDTR &= 0xFFFFFFF0;
		mb->TDTR |= frame->can_dlc & 0x0F;

		mb->TDLR =
			  ( frame->data[3] << 24 )
			| ( frame->data[2] << 16 )
			| ( frame->data[1] <<  8 )
			| ( frame->data[0] <<  0 );

		mb->TDHR =
			  ( frame->data[7] << 24 )
			| ( frame->data[6] << 16 )
			| ( frame->data[5] <<  8 )
			| ( frame->data[4] <<  0 );

		/* request transmission */
		mb->TIR |= CAN_TI0R_TXRQ;

		return true;
	} else {
		return false;
	}

#elif defined(STM32G0)

    _hcan = hcan;
    /* Check that the Tx FIFO/Queue is not full */
    if((hfdcan2.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U)
    {
		return false;
    }
	
	TxHeader.Identifier = frame->can_id & 0x7FF;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = (uint32_t)((frame->can_dlc & 0x0F)<<16);;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, frame->data);
	
	return true;

#endif

}

uint32_t can_get_error_status(can_data_t *hcan)
{
#if defined(STM32F0) || defined(STM32F4)
	CAN_TypeDef *can = hcan->instance;
	return can->ESR;
#elif defined(STM32G0)

	uint32_t esr_data=0x00000000;

    _hcan = hcan;

	esr_data |=(uint32_t)(((FDCAN2->PSR & 0x00000040)>>6)<<0);//EW
	esr_data |=(uint32_t)(((FDCAN2->PSR & 0x00000020)>>5)<<1);//EP
	esr_data |=(uint32_t)(((FDCAN2->PSR & 0x00000080)>>7)<<2);//BO
	esr_data |=(uint32_t)(((FDCAN2->PSR & 0x00000007)>>0)<<4);//LEC

	esr_data |=(uint32_t)(((FDCAN2->ECR & 0x000000FF)>>0)<<16);//TEC
	esr_data |=(uint32_t)(((FDCAN2->ECR & 0x00007F00)>>8)<<24);//REC
	
	return  esr_data;
#endif
}

uint32_t _err;
static bool status_is_active(uint32_t err)
{
#if defined(STM32F0) || defined(STM32F4)
	return !(err & (CAN_ESR_BOFF | CAN_ESR_EPVF));
#elif defined(STM32G0)
	_err=err;
	return !(FDCAN2->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP));
#endif
}

bool can_parse_error_status(uint32_t err, uint32_t last_err, can_data_t *hcan, struct gs_host_frame *frame)
{
	/* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;

#if defined(STM32G0)
    _hcan = hcan;
#endif

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;

	if (err & CAN_ESR_BOFF) {
		frame->can_id |= CAN_ERR_BUSOFF;
		if (!(last_err & CAN_ESR_BOFF)) {
			/* We transitioned to bus-off. */
			should_send = true;
		}
	} else if (last_err & CAN_ESR_BOFF) {
		/* We transitioned out of bus-off. */
		should_send = true;
	}

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

	uint8_t tx_error_cnt = (err>>16) & 0xFF;
	uint8_t rx_error_cnt = (err>>24) & 0xFF;
	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->data[6] = tx_error_cnt;
	frame->data[7] = rx_error_cnt;

	uint8_t last_tx_error_cnt = (last_err>>16) & 0xFF;
	uint8_t last_rx_error_cnt = (last_err>>24) & 0xFF;
	/* If either error counter transitioned to/from 0. */
	if ((tx_error_cnt == 0) != (last_tx_error_cnt == 0)) {
		should_send = true;
	}
	if ((rx_error_cnt == 0) != (last_rx_error_cnt == 0)) {
		should_send = true;
	}
	/* If either error counter increased by 15. */
	if (((int)last_tx_error_cnt + CAN_ERRCOUNT_THRESHOLD) < tx_error_cnt) {
		should_send = true;
	}
	if (((int)last_rx_error_cnt + CAN_ERRCOUNT_THRESHOLD) < rx_error_cnt) {
		should_send = true;
	}

	if (err & CAN_ESR_EPVF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
		if (!(last_err & CAN_ESR_EPVF)) {
			should_send = true;
		}
	} else if (err & CAN_ESR_EWGF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
		if (!(last_err & CAN_ESR_EWGF)) {
			should_send = true;
		}
	} else if (last_err & (CAN_ESR_EPVF | CAN_ESR_EWGF)) {
		should_send = true;
	}

	uint8_t lec = (err>>4) & 0x07;
	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}

#if defined(STM32F0) || defined(STM32F4)
	CAN_TypeDef *can = hcan->instance;
	/* Write 7 to LEC so we know if it gets set to the same thing again */
	can->ESR = 7<<4;
#elif defined(STM32G0)
	FDCAN2->ECR = 0x0000;
	FDCAN2->PSR = 0x0007;	
#endif

	return should_send;
}
