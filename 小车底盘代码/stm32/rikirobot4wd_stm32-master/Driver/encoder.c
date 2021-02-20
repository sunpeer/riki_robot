#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"

int en_pos1 = 0;
int en_pos2 = 0;
int en_pos3 = 0;
int en_pos4 = 0;

void encoder_init(Encoder_TypeDef encoder)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; 

	if(encoder == ENCODER1) {
		RCC_APB2PeriphClockCmd(RIKI_ENCODER1_GPIO_CLK , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitStructure.GPIO_Pin = RIKI_ENCODER1_A_PIN | RIKI_ENCODER1_B_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(RIKI_ENCODER1_GPIO_PORT, &GPIO_InitStructure);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER1_A_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER1_B_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		EXTI_InitStructure.EXTI_Line = RIKI_ENCODER1_A_EXTI_LINE | RIKI_ENCODER1_B_EXTI_LINE ;  
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
		EXTI_Init(&EXTI_InitStructure);  
		GPIO_EXTILineConfig(RIKI_ENCODER1_A_EXTI_PORT, RIKI_ENCODER1_A_EXTI_PIN);  
		GPIO_EXTILineConfig(RIKI_ENCODER1_B_EXTI_PORT, RIKI_ENCODER1_B_EXTI_PIN);  
	}

	if(encoder == ENCODER2){
		RCC_APB2PeriphClockCmd(RIKI_ENCODER2_GPIO_CLK, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitStructure.GPIO_Pin = RIKI_ENCODER2_A_PIN | RIKI_ENCODER2_B_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(RIKI_ENCODER2_GPIO_PORT, &GPIO_InitStructure);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER2_A_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER2_B_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		EXTI_InitStructure.EXTI_Line = RIKI_ENCODER2_A_EXTI_LINE | RIKI_ENCODER2_B_EXTI_LINE ;  
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
		EXTI_Init(&EXTI_InitStructure);  

		GPIO_EXTILineConfig(RIKI_ENCODER2_A_EXTI_PORT, RIKI_ENCODER2_A_EXTI_PIN);  
		GPIO_EXTILineConfig(RIKI_ENCODER2_B_EXTI_PORT, RIKI_ENCODER2_B_EXTI_PIN);  
	}	

	if(encoder == ENCODER3){
		RCC_APB2PeriphClockCmd(RIKI_ENCODER2_GPIO_CLK, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitStructure.GPIO_Pin = RIKI_ENCODER3_A_PIN | RIKI_ENCODER3_B_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(RIKI_ENCODER3_GPIO_PORT, &GPIO_InitStructure);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER3_A_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER3_B_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		EXTI_InitStructure.EXTI_Line = RIKI_ENCODER3_A_EXTI_LINE | RIKI_ENCODER3_B_EXTI_LINE ;  
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
		EXTI_Init(&EXTI_InitStructure);  

		GPIO_EXTILineConfig(RIKI_ENCODER3_A_EXTI_PORT, RIKI_ENCODER3_A_EXTI_PIN);  
		GPIO_EXTILineConfig(RIKI_ENCODER3_B_EXTI_PORT, RIKI_ENCODER3_B_EXTI_PIN);  
	}	

	if(encoder == ENCODER4){
		RCC_APB2PeriphClockCmd(RIKI_ENCODER4_GPIO_CLK, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		GPIO_InitStructure.GPIO_Pin = RIKI_ENCODER4_A_PIN | RIKI_ENCODER4_B_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(RIKI_ENCODER4_GPIO_PORT, &GPIO_InitStructure);

		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER4_A_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = RIKI_ENCODER4_B_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
		NVIC_Init(&NVIC_InitStructure);

		EXTI_InitStructure.EXTI_Line = RIKI_ENCODER4_A_EXTI_LINE | RIKI_ENCODER4_B_EXTI_LINE ;  
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
		EXTI_Init(&EXTI_InitStructure);  

		GPIO_EXTILineConfig(RIKI_ENCODER4_A_EXTI_PORT, RIKI_ENCODER4_A_EXTI_PIN);  
		GPIO_EXTILineConfig(RIKI_ENCODER4_B_EXTI_PORT, RIKI_ENCODER4_B_EXTI_PIN);  
	}	
}

void EXTI0_IRQHandler(void)  
{
	if(EXTI_GetITStatus(RIKI_ENCODER2_A_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_A_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_B_PIN) == LOW){
				en_pos2++;
			}else {
				en_pos2--;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_B_PIN) == HIGH){
				en_pos2++;
			}else {
				en_pos2--;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER2_A_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER2_A_EXTI_LINE);
	}
}	

void EXTI1_IRQHandler(void)  
{  
	if(EXTI_GetITStatus(RIKI_ENCODER2_B_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_B_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_A_PIN) == HIGH){
				en_pos2++;
			}else {
				en_pos2--;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER2_GPIO_PORT, RIKI_ENCODER2_A_PIN) == LOW){
				en_pos2++;
			}else {
				en_pos2--;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER2_B_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER2_B_EXTI_LINE);
	}
}	

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(RIKI_ENCODER1_A_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_A_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_B_PIN) == LOW){
				en_pos1--;
			}else {
				en_pos1++;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_B_PIN) == HIGH){
				en_pos1--;
			}else {
				en_pos1++;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER1_A_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER1_A_EXTI_LINE);
	}

	if(EXTI_GetITStatus(RIKI_ENCODER1_B_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_B_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_A_PIN) == HIGH){
				en_pos1--;
			}else {
				en_pos1++;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER1_GPIO_PORT, RIKI_ENCODER1_A_PIN) == LOW){
				en_pos1--;
			}else {
				en_pos1++;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER1_B_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER1_B_EXTI_LINE);
	}

	if(EXTI_GetITStatus(RIKI_ENCODER3_A_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_A_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_B_PIN) == LOW){
				en_pos3++;
			}else {
				en_pos3--;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_B_PIN) == HIGH){
				en_pos3++;
			}else {
				en_pos3--;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER3_A_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER3_A_EXTI_LINE);
	}

	if(EXTI_GetITStatus(RIKI_ENCODER3_B_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_B_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_A_PIN) == HIGH){
				en_pos3++;
			}else {
				en_pos3--;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER3_GPIO_PORT, RIKI_ENCODER3_A_PIN) == LOW){
				en_pos3++;
			}else {
				en_pos3--;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER3_B_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER3_B_EXTI_LINE);
	}


}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(RIKI_ENCODER4_A_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_A_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_B_PIN) == LOW){
				en_pos4--;
			}else {
				en_pos4++;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_B_PIN) == HIGH){
				en_pos4--;
			}else {
				en_pos4++;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER4_A_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER4_A_EXTI_LINE);
	}

	if(EXTI_GetITStatus(RIKI_ENCODER4_B_EXTI_LINE) != RESET){
		if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_B_PIN) == HIGH){
			if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_A_PIN) == HIGH){
				en_pos4--;
			}else {
				en_pos4++;
			}
		} else {
			if(GPIO_ReadInputDataBit(RIKI_ENCODER4_GPIO_PORT, RIKI_ENCODER4_A_PIN) == LOW){
				en_pos4--;
			}else {
				en_pos4++;
			}
		}
		EXTI_ClearITPendingBit(RIKI_ENCODER4_B_EXTI_LINE);
		EXTI_ClearFlag(RIKI_ENCODER4_B_EXTI_LINE);
	}

}

#ifdef __cplusplus
}
#endif
