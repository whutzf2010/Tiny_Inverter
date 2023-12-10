/**
  ******************************************************************************
  * @file    	stm32f3xx_it.c
  * @brief   	Interrupt Service Routines.
	* @Author   whutzf2010
  ******************************************************************************

  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
#include "arm_math.h"	// 

/* Private typedef -----------------------------------------------------------*/
ACStateTypedef ACState;
int16_t u16VacState;

ModeTypedef Mode;
int16_t u16Mode;

OnGridStateTypedef OnGridState;
int16_t u16OnGridState;

DQStructTypedef OnGridDQ;
PIDStructTypedef OffGridPID;
PIDStructTypedef OnGridPID;
PRStructTypedef OnGridPR;
SOGIPLLStructTypedef SOGIPLL;

//UARTStructTypeDef SciCom;

int16_t TIM2_Counter;
int16_t TIM2_Delay_Counter; //10KHz 

float VacRmsVal;
int16_t VacPeakVal;
float SinVal;
float f32VacVal;
float f32VacSin;
int16_t u16VacVal;
int16_t u16VacAbsVal;
int16_t u16VacFreqCounter;
float f32VacFreqCounter;
float f32VacFreq;
double VacTimer;
float VacGain;
int16_t u16PwmUpdate;
int16_t u16PwmUpdateCH2;
int16_t u16PwmUpdateCH3;
float f32PwmUpdate;

uint8_t DataTailSendTx;
uint8_t USARTBufferTx[16];
uint8_t USARTBufferRx[12];
uint8_t *USARTf32TxADDr;
float USARTf32Tx[4];

/* RMS Value --------------------------------------------------------*/
float f32MaxValue;
float f32MaxValueBefore;
float f32MinValue;
float f32MinValueBefore;


//**************************************************************//
//	     	Digital Power Control Algrithum		            	//
//						whutzf2010																			//
//						2023.July.17th 															//
//**************************************************************//


void TIM2_IRQHandler(void)
{
	
	if(LL_TIM_IsActiveFlag_UPDATE(TIM2))
	{
		LL_TIM_ClearFlag_UPDATE(TIM2);	


//---------VOFA Transmission Data--------------//
			
		USARTf32Tx[0] = f32_VAC;//u16PwmUpdateCH2;//OnGridPR.Out0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//f32_VAC_filtered;//SOGIPLL.theta;//
		USARTf32Tx[1] = f32_VIN;//OnGridPR.Err0;//arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//SOGIPLL.alpha0;//f32_IAC_Ref;
		USARTf32Tx[2] = -f32_IAC;//- f32_VIN_filtered;//SOGIPLL.beta0;//f32_VAC_filtered;
		//USARTf32Tx[3] = arm_cos_f32(SOGIPLL.theta)*OnGridPID.Out0_Vol;//0;
			
//----------VOFA USART Communication Proctol-2kHz--------------//			
		USARTf32TxADDr = (uint8_t *)&USARTf32Tx;
		USARTBufferTx[0] = USARTf32TxADDr[0];
		USARTBufferTx[1] = USARTf32TxADDr[1];
		USARTBufferTx[2] = USARTf32TxADDr[2];
		USARTBufferTx[3] = USARTf32TxADDr[3];
			
		USARTBufferTx[4] = USARTf32TxADDr[4];
		USARTBufferTx[5] = USARTf32TxADDr[5];
		USARTBufferTx[6] = USARTf32TxADDr[6];
		USARTBufferTx[7] = USARTf32TxADDr[7];
		
		USARTBufferTx[8] = USARTf32TxADDr[8];
		USARTBufferTx[9] = USARTf32TxADDr[9];
		USARTBufferTx[10] = USARTf32TxADDr[10];
		USARTBufferTx[11] = USARTf32TxADDr[11];
		
		//USARTBufferTx[12] = USARTf32TxADDr[12];
		//USARTBufferTx[13] = USARTf32TxADDr[13];
		//USARTBufferTx[14] = USARTf32TxADDr[14];
		//USARTBufferTx[15] = USARTf32TxADDr[15];
		//}	
//------------ADC Sampling Result Update--3%---------------//	
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);			
		ADCValue_VIN = aADCxConvertedValues[0]; //VIN
		ADCValue_IAC = aADCxConvertedValues[1]; //IAC
		ADCValue_VACL = aADCxConvertedValues[2]; //VACL
		ADCValue_VACN =  aADCxConvertedValues[3]; //VACN
		ADCValue_IIN = 	aADCxConvertedValues[4];  //IIN
		
		f32_VIN = (float)ADCValue_VIN;
		f32_VIN = f32_VIN * 0.016; //  f32_VIN = ADCValue_VIN/4096*3.3*21;
		f32_VIN = f32_VIN * 1.0636; // calibrate
		f32_VIN += 1.0882;          // calibrate
		if(f32_VIN <= 0.01)
			f32_VIN = 0.01;
		
    f32_VAC = (float)(ADCValue_VACL - ADCValue_VACN);
		f32_VAC = f32_VAC * 0.019; //  f32_VAC = ADCValue_VAC/4096*3.3*21;	
		//f32_VAC += 0.0; 
		//f32_VAC = f32_VAC * (1.168); 
		
		f32_IIN = (float)ADCValue_IIN;
		f32_IIN = f32_IIN * 0.004; //  f32_IIN = ADCValue_IIN/4096*3.3*5;
		
		
		f32_IAC = (float)(ADCValue_IAC-1462);//1440);//1450);
		f32_IAC = f32_IAC * (-0.00644);//0.00644; //  f32_IAC = ADCValue_IAC/4096*3.3*8;
			
		//TIM_OC_InitStruct.CompareValue = 2000;	
		//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);	
		//}

		
//-------- Low Pass Filter ---------------//
		f32_VAC_filter = f32_VAC * VAC_FILTER_K;
		f32_VAC_filter += f32_VAC_filtered * VAC_FILTER_1_K;
		f32_VAC_filtered = f32_VAC_filter;
	
		f32_VIN_filter = f32_VIN * VIN_FILTER_K;
		f32_VIN_filter += f32_VIN_filtered * VIN_FILTER_1_K;
		f32_VIN_filtered = f32_VIN_filter;	
		
		f32_IAC_filter = f32_IAC * IAC_FILTER_K;
		f32_IAC_filter += f32_IAC_filtered * IAC_FILTER_1_K;
		f32_IAC_filtered = f32_IAC_filter;
		//if(f32_VAC_filtered > 15.0)
		//	f32_VAC_filtered = 15.0;
		//if(f32_VAC_filtered < -15.0)
		//	f32_VAC_filtered = -15.0;	
		//if(f32_VAC >= 0.0)
		//	f32MaxValueBefore = f32_VAC;
		//else 
		//	f32MinValueBefore = f32_VAC;
		
//----------- MAX/MIN Value ------------------//			
		//if(f32MinValue >= f32_VAC_filtered)
		//	f32MinValue = f32_VAC_filtered;
		//if(f32MaxValue <= f32_VAC_filtered)
		//	f32MaxValue = f32_VAC_filtered;
		
//--------RMS Value Calculation---------------------//
		TIM2_Counter++;
		if(TIM2_Counter == 4)
		{
			TIM2_Counter = 0;
			
			f32_VAC_Rms_Counter ++;
			if(f32_VAC_Rms_Counter == 50) 
				f32_VAC_Rms_Counter = 0;
			f32_VAC_Rms[f32_VAC_Rms_Counter] = (float)(f32_VAC*f32_VAC);
			f32SUMValue = 0.0;
			int16_t i=0;
			while(i<=50)
			{
				f32SUMValue += f32_VAC_Rms[i];
				i++;
			}
			f32SUMValue = f32SUMValue/50.0;
			f32RMSValue = sqrtf(f32SUMValue);
			if(f32RMSValue <= 0.5)
				f32RMSValue = 0.5;
			else if(f32RMSValue >= 30.1)
				f32RMSValue = 30.1;			
		}
		
//**************************************************************//
//		Digital Power Off Grid Unidirectional AC-DC Mode		//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//
		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_13);		//GPIOB-13 LED2	

			if(u16Mode == InvertOffGrid)
			{
				
			
//-------------------AC Soft Start --------------------------//	
//				f32_VAC_Set_Update += f32_VAC_Set_Step;
//				if(f32_VAC_Set_Update >= f32_VAC_Set)
//					f32_VAC_Set_Update = f32_VAC_Set;
//				f32_VAC_Gain = f32_VAC_Set_Update/f32_VIN;
		
//--------------Voltage Loop PI Controller----------------------//
				//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);					
				OffGridPID.Err0_Vol = f32_VRms_Set - f32RMSValue;
				OffGridPID.Out0_Vol = OffGridPID.Out1_Vol + (float)(VAC_KP * OffGridPID.Err0_Vol); 
				OffGridPID.Out0_Vol = OffGridPID.Out0_Vol + (float)(VAC_KI * OffGridPID.Err0_Vol); 
				OffGridPID.Out0_Vol = OffGridPID.Out0_Vol - (float)(VAC_KP * OffGridPID.Err1_Vol) ;


				if(OffGridPID.Out0_Vol > 2.0)
					OffGridPID.Out0_Vol  = 2.0;
				else if(OffGridPID.Out0_Vol < 0.0)
					OffGridPID.Out0_Vol = 0.0;
				
				//if(PID.Out1_Vol >= 2.0)
				//	PID.Out1_Vol  = 2.0;
				//else if(PID.Out1_Vol < 0.0)
				//	PID.Out1_Vol = 0.0;
		
				OffGridPID.Err1_Vol = OffGridPID.Err0_Vol;
				OffGridPID.Out1_Vol = OffGridPID.Out0_Vol;
			
//				f32_VAC_Gain = (float) (f32_VAC_Gain *PID.Out0_Vol);
				f32_VAC_Gain = OffGridPID.Out0_Vol;
				if(f32_VAC_Gain >= 1.0)
					f32_VAC_Gain  = 1.0;
				else if(f32_VAC_Gain  < 0.2)
					f32_VAC_Gain  = 0.2;
			
//--------------- Current Loop PI Controller------------------//				
				//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);		

/*
				f32_IAC_Ref = PID.Out0_Vol/f32RMSValue;// 
				f32_IAC_Ref = f32_IAC_Ref/1;// 
				f32_IAC_Ref = f32_IAC_Ref *SinVal;
				//f32_IAC_Ref = fabs(f32_IAC_Ref);
				//f32_IAC_filtered = fabs(f32_IAC_filtered);
				if(f32_IAC_Ref > 0.0)
					f32_IAC_Ref = f32_IAC_Ref;
				else if(f32_IAC_Ref <= 0.0)
					f32_IAC_Ref = -f32_IAC_Ref;
				if(f32_IAC_filtered > 0.0)
					f32_IAC_filtered = f32_IAC_filtered;
				else if(f32_IAC_filtered <= 0.0)
					f32_IAC_filtered = -f32_IAC_filtered;			
				//LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);					
				PID.Err0_Cur = f32_IAC_Ref - f32_IAC_filtered;
				if(PID.Err0_Cur >10.0)  // Avoid PIout NaN
					PID.Err0_Cur = 10.0;
				else if(PID.Err0_Cur <-10.0)
					PID.Err0_Cur = -10.0;
				PID.Out0_Cur = PID.Out1_Cur + (float)(IAC_KP * PID.Err0_Cur); 
				PID.Out0_Cur = PID.Out0_Cur + (float)(IAC_KI * PID.Err0_Cur); 
				PID.Out0_Cur = PID.Out0_Cur - (float)(IAC_KP * PID.Err1_Cur);

				if(PID.Out0_Cur > 1.0)
					PID.Out0_Cur  = 1.0;
				else if(PID.Out0_Cur < 0.0)
					PID.Out0_Cur = 0.0;

				f32VacVal = VacPeakVal*PID.Out0_Cur;
				PID.Err1_Cur = PID.Err0_Cur;
				PID.Out1_Cur = PID.Out0_Cur;
	*/		
//==================== SPWM Modulation ======================//
				u16VacFreqCounter ++;
				if (u16VacFreqCounter == INVERT_AC_FREQ)
				{
						u16VacFreqCounter = 0;
				}
				f32VacFreqCounter = (float)u16VacFreqCounter;
				f32VacFreq = (float)INVERT_AC_FREQ;
				VacTimer = f32VacFreqCounter/f32VacFreq;
				SinVal = arm_sin_f32(6.2831852*(float)VacTimer);
				f32VacVal = VacPeakVal*SinVal;
				//u16PwmUpdate = (int16_t)VacVal;
				
				f32VacVal = (float)(f32VacVal*f32_VAC_Gain);
				f32VacVal =  f32VacVal/2;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

//				if (f32VacVal >= 0.0)
//				{
					u16PwmUpdateCH2 = (int16_t)(f32VacVal+INVERT_PWM_HALF);
					u16PwmUpdateCH3 = (int16_t)(f32VacVal+INVERT_PWM_HALF);
//				}
//				else
//				{
//						u16PwmUpdateCH2 = PWM_20KHZ_HALF-u16VacVal;
//						u16PwmUpdateCH3 = (int16_t)(VacPeakVal + f32VacVal);	
//				}
				LL_TIM_OC_SetCompareCH2(TIM1, u16PwmUpdateCH2);
				LL_TIM_OC_SetCompareCH3(TIM1, u16PwmUpdateCH3);	
		


			}	
	
//**************************************************************//
//		Digital Power On Grid Bidirectional AC-DC Mode			//
//						whutzf2010									//
//						2023.July.17th 							//
//**************************************************************//
//====================Full HF SPWM Modulation ======================//
	  	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_12);		//GPIOB-12 LED3		


		
	}	



void USART2_IRQHandler(void)
{		
   
	LL_USART_ClearFlag_TC(USART2);
	//if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
   //{
    //UsartRxValue = (uint16_t)LL_USART_ReceiveData8(USART2);		
   //}
     
}




/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
