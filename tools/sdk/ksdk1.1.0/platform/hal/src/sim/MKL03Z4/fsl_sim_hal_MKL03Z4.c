/*
* Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_sim_hal.h"

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* APIs
******************************************************************************/

/*FUNCTION**********************************************************************
*
* Function Name : CLOCK_HAL_SetOutDiv
* Description   : Set all clock out dividers setting at the same time
* This function will set the setting for all clock out dividers.
*
*END**************************************************************************/
void CLOCK_HAL_SetOutDiv(uint32_t baseAddr,
                         uint8_t outdiv1,
                         uint8_t outdiv2,
                         uint8_t outdiv3,
                         uint8_t outdiv4)
{
    uint32_t clkdiv1 = 0;
    
    clkdiv1 |= BF_SIM_CLKDIV1_OUTDIV1(outdiv1);
    clkdiv1 |= BF_SIM_CLKDIV1_OUTDIV4(outdiv4);
    
    HW_SIM_CLKDIV1_WR(baseAddr, clkdiv1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_HAL_GetOutDiv
 * Description   : Get all clock out dividers setting at the same time
 * This function will get the setting for all clock out dividers.
 *
 *END**************************************************************************/
void CLOCK_HAL_GetOutDiv(uint32_t baseAddr,
                         uint8_t *outdiv1,
                         uint8_t *outdiv2,
                         uint8_t *outdiv3,
                         uint8_t *outdiv4)
{
    *outdiv1 = BR_SIM_CLKDIV1_OUTDIV1(baseAddr);
    *outdiv2 = 0U;
    *outdiv3 = 0U;
    *outdiv4 = BR_SIM_CLKDIV1_OUTDIV4(baseAddr);
}


/*FUNCTION**********************************************************************
*
* Function Name : SIM_HAL_SetAdcTriggerModeOneStep
* Description   : Set ADCx trigger setting.
* This function sets ADC alternate trigger, pre-trigger mode and trigger mode.
*
*END**************************************************************************/
void SIM_HAL_SetAdcTriggerModeOneStep(uint32_t baseAddr,
                                      uint32_t instance,
                                      bool    altTrigEn,
                                      sim_adc_pretrg_sel_t preTrigSel,
                                      sim_adc_trg_sel_t trigSel)
{
    assert(instance < HW_ADC_INSTANCE_COUNT);
    
    BW_SIM_SOPT7_ADC0ALTTRGEN(baseAddr, altTrigEn ? 1 : 0);
    BW_SIM_SOPT7_ADC0PRETRGSEL(baseAddr, preTrigSel);
    
    if (altTrigEn)
    {
        BW_SIM_SOPT7_ADC0TRGSEL(baseAddr, trigSel);
    }
    
}

/*FUNCTION**********************************************************************
*
* Function Name : SIM_HAL_SetTpmExternalClkPinSelMode
* Description	 : Set Timer/PWM x external clock pin select setting 
* This function will select the source of Timer/PWM x external clock pin select
* 
*END**************************************************************************/
void SIM_HAL_SetTpmExternalClkPinSelMode(uint32_t baseAddr,
                                         uint32_t instance,
                                         sim_tpm_clk_sel_t select)
{
    assert (instance < HW_TPM_INSTANCE_COUNT);
    
    switch (instance)
    {
    case 0:
        BW_SIM_SOPT4_TPM0CLKSEL(baseAddr, select);
        break;
    case 1:
        BW_SIM_SOPT4_TPM1CLKSEL(baseAddr, select);
        break;
    default:
        break;
    }
}

/*FUNCTION**********************************************************************
*
* Function Name : SIM_HAL_GetTpmExternalClkPinSelMode
* Description	 : Get Timer/PWM x external clock pin select setting
* This function will get Timer/PWM x external clock pin select setting.
* 
*END**************************************************************************/
sim_tpm_clk_sel_t SIM_HAL_GetTpmExternalClkPinSelMode(uint32_t baseAddr, uint32_t instance)
{
    sim_tpm_clk_sel_t retValue = (sim_tpm_clk_sel_t)0;
    
    assert (instance < HW_TPM_INSTANCE_COUNT);
    
    switch (instance)
    {
    case 0:
        retValue = (sim_tpm_clk_sel_t)BR_SIM_SOPT4_TPM0CLKSEL(baseAddr);
        break;
    case 1:
        retValue = (sim_tpm_clk_sel_t)BR_SIM_SOPT4_TPM1CLKSEL(baseAddr);
        break;
    default:
        break;
    }
    
    return retValue;
}

/*******************************************************************************
* EOF
******************************************************************************/
