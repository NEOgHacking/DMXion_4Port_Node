
/*
 * Copyright (c) 2021 Jostein Løwer 
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxOutput.h"
#include "DmxOutput.pio.h"

#if defined(ARDUINO_ARCH_MBED)
  #include <clocks.h>
  #include <irq.h>
#else
  #include "hardware/clocks.h"
  #include "hardware/irq.h"
#endif

DmxOutput::return_code DmxOutput::begin(uint pin, PIO pio)
{
#if PICO_PIO_USE_GPIO_BASE
    if (pin >= 48)
    {
        return ERR_INVALID_PIN;
    }

    uint gpio_base = pin >= 32 ? 16 : 0;
    if (pin >= 16 && pin < 32 && pio_get_gpio_base(pio) == 16)
    {
        gpio_base = 16;
    }

    if (pio_get_gpio_base(pio) != gpio_base &&
        pio_set_gpio_base(pio, gpio_base) != PICO_OK)
    {
        return ERR_GPIO_BASE_UNAVAILABLE;
    }
#else
    if (pin >= 32)
    {
        return ERR_INVALID_PIN;
    }
#endif

    /* 
    Attempt to load the DMX PIO assembly program 
    into the PIO program memory
    */

    if (!pio_can_add_program(pio, &DmxOutput_program))
    {
        return ERR_INSUFFICIENT_PRGM_MEM;
    }
    uint prgm_offset = pio_add_program(pio, &DmxOutput_program);

    /* 
    Attempt to claim an unused State Machine 
    into the PIO program memory
    */

    int sm = pio_claim_unused_sm(pio, false);
    if (sm == -1)
    {
        pio_remove_program(pio, &DmxOutput_program, prgm_offset);
        return ERR_NO_SM_AVAILABLE;
    }

    // Set this pin's GPIO function (connect PIO to the pad)
#if PICO_PIO_USE_GPIO_BASE
    uint64_t pin_mask = 1ull << pin;
    pio_sm_set_pins_with_mask64(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pindirs_with_mask64(pio, sm, pin_mask, pin_mask);
#else
    uint32_t pin_mask = 1u << pin;
    pio_sm_set_pins_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
#endif
    pio_gpio_init(pio, pin);

    // Generate the default PIO state machine config provided by pioasm
    pio_sm_config sm_conf = DmxOutput_program_get_default_config(prgm_offset);

    // Setup the side-set pins for the PIO state machine
    sm_config_set_out_pins(&sm_conf, pin, 1);
    sm_config_set_sideset_pins(&sm_conf, pin);

    // Setup the clock divider to run the state machine at exactly 1MHz
    uint clk_div = clock_get_hz(clk_sys) / DMX_SM_FREQ;
    sm_config_set_clkdiv(&sm_conf, clk_div);

    // Load our configuration, jump to the start of the program and run the State Machine
    pio_sm_init(pio, sm, prgm_offset, &sm_conf);
    pio_sm_set_enabled(pio, sm, true);

    // Claim an unused DMA channel.
    // The channel is kept througout the lifetime of the DMX source
    int dma = dma_claim_unused_channel(false);

    if (dma == -1)
    {
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_unclaim(pio, sm);
        pio_remove_program(pio, &DmxOutput_program, prgm_offset);
        return ERR_NO_DMA_AVAILABLE;
    }

    // Get the default DMA config for our claimed channel
    dma_channel_config dma_conf = dma_channel_get_default_config(dma);

    // Set the DMA to move one byte per DREQ signal
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);

    // Setup the DREQ so that the DMA only moves data when there
    // is available room in the TXF buffer of our PIO state machine
    channel_config_set_dreq(&dma_conf, pio_get_dreq(pio, sm, true));

    // Setup the DMA to write to the TXF buffer of the PIO state machine
    dma_channel_set_write_addr(dma, &pio->txf[sm], false);

    // Apply the config
    dma_channel_set_config(dma, &dma_conf, false);

    // Set member values of C++ class
    _prgm_offset = prgm_offset;
    _pio = pio;
    _sm = sm;
    _pin = pin;
    _dma = dma;

    return SUCCESS;
}

void DmxOutput::write(uint8_t *universe, uint length)
{

    // Temporarily disable the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Reset the PIO state machine to a consistent state. Clear the buffers and registers
    pio_sm_restart(_pio, _sm);

    // Start the DMX PIO program from the beginning
    pio_sm_exec(_pio, _sm, pio_encode_jmp(_prgm_offset));

    // Restart the PIO state machinge
    pio_sm_set_enabled(_pio, _sm, true);

    // Start the DMA transfer
    dma_channel_transfer_from_buffer_now(_dma, universe, length);
}

bool DmxOutput::busy()
{
    if (dma_channel_is_busy(_dma))
        return true;

    return !pio_sm_is_tx_fifo_empty(_pio, _sm);
}

/*
void Dmx::await()
{
    dma_channel_wait_for_finish_blocking(_dma);

    while (!pio_sm_is_tx_fifo_empty(_pio, _sm))
    {
    }
}
*/

void DmxOutput::end()
{
    // Stop the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Remove the PIO DMX program from the PIO program memory
    pio_remove_program(_pio, &DmxOutput_program, _prgm_offset);

    // Unclaim the DMA channel
    dma_channel_unclaim(_dma);

    // Unclaim the sm
    pio_sm_unclaim(_pio, _sm);
}
