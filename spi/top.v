// =============================================================================
// top_bringup.v  —  Bringup loopback + Custom SPI routine for LFE5U-25F BGA256
//
// Reflects 4 signals from CN1 (ADC board interface) directly to J6 pins 3-6.
// When btn_trigger (N13) goes low, runs an SPI-like clocking routine on 1.5V pins.
// =============================================================================

module top (
    input  wire        clk_16m,      // A7  — 16 MHz crystal

    // CN1 — ADC board inputs (Configured for 1.5V operations)
    input  wire        cb_d1,        // B15 — serial data lane 1
    input  wire        cb_clk32mhz,  // K16 — 32 MHz bit clock from ADC board
    input  wire        cb_read,      // J13 — next_amps group-boundary pulse
    input  wire        cb_sync,      // H14 — frame sync

    // Status LED
    output wire        led_power,    // P16 → D1: ~1 Hz blink from 16 MHz crystal

    // J6 — loopback outputs
    output wire        j6_d1,        // A15 — pin 3
    output wire        j6_read,      // A14 — pin 4
    output wire        j6_sync,      // B14 — pin 5
    output wire        j6_clk,       // A13 — pin 6
    output wire        j6_clkref,    // A12 — pin 7

    // Added Trigger & 1.5V SPI Controller Interface
    input  wire        btn_trigger,  // N13 — Active Low Button
    output wire        spi_sig1,     // F15
    output wire        spi_sig2,     // F14
    output wire        spi_clk,      // E14 — Customizable frequency clock
    output wire        spi_latch     // F16 — Quick end pulse
);

    // -------------------------------------------------------------------------
    // 1. Original Loopback & Blink Logic
    // -------------------------------------------------------------------------
    reg [23:0] ctr = 24'd0;
    always @(posedge clk_16m)
        ctr <= ctr + 1'b1;

    assign led_power = ctr[23];

    assign j6_d1     = cb_d1;
    assign j6_read   = cb_read;
    assign j6_sync   = cb_sync;
    assign j6_clk    = cb_clk32mhz;
    assign j6_clkref = cb_clk32mhz;

    // -------------------------------------------------------------------------
    // 2. Configuration Parameters
    // -------------------------------------------------------------------------
    // Frequency Calculation: SPI_CLK = 16 MHz / CLK_DIVIDER.
    // Examples: 
    //   - CLK_DIVIDER = 16 -> 1.0 MHz SPI clock
    //   - CLK_DIVIDER = 8  -> 2.0 MHz SPI clock
    // Note: CLK_DIVIDER must be an even integer >= 2 to ensure a 50% duty cycle.
    parameter CLK_DIVIDER = 16; 

    // -------------------------------------------------------------------------
    // 3. State Machine & Signal Generation
    // -------------------------------------------------------------------------
    // Define unique binary encodings for our 4-state control pipeline
    localparam ST_IDLE    = 2'b00; // Waiting for button press trigger
    localparam ST_RUNNING = 2'b01; // Generating SPI clock cycles
    localparam ST_LATCH   = 2'b10; // Pulsing the latch signal high for 1 full SPI period
    localparam ST_DONE    = 2'b11; // Holding final safe-state indefinitely

    reg [1:0]  state               = ST_IDLE;
    reg [7:0]  clk_half_period_ctr = 0; // Counts master clock pulses to time the SPI half-period
    reg        latch_phase         = 1'b0;  // High during the second half of the latch period
    
    // 12-bit cycle counter. It counts from 0 to 4095 (total of 4096 full cycles).
    // Using a clean power-of-two (4096) completely clears the "at least 2500" rule 
    // and eliminates complex logic by letting us look for a simple rollover value (12'hFFF).
    reg [11:0] spi_cycle_ctr       = 0; 

    // Internal registers tracking the state of our 1.5V output pins
    reg out_sig1  = 1'b1; // Default high until trigger occurs
    reg out_sig2  = 1'b1; // Default high until trigger occurs
    reg out_clk   = 1'b0; // Default low
    reg out_latch = 1'b0; // Default low

    always @(posedge clk_16m) begin
        case (state)
            
            // -----------------------------------------------------------------
            // ST_IDLE: Sit here doing nothing until the button pulls N13 low.
            // -----------------------------------------------------------------
            ST_IDLE: begin
                out_latch   <= 1'b0; // Explicitly ensure latch is resting low
                latch_phase <= 1'b0;
                
                if (btn_trigger == 1'b0) begin
                    // Trigger detected! Instantly drop the signal lines low
                    out_sig1            <= 1'b0; 
                    out_sig2            <= 1'b0;
                    out_clk             <= 1'b0;
                    
                    // Reset our execution counters to zero
                    clk_half_period_ctr <= 0;
                    spi_cycle_ctr       <= 0;
                    
                    // Advance to the clock generation state
                    state               <= ST_RUNNING;
                end
            end

            // -----------------------------------------------------------------
            // ST_RUNNING: Divides the master clock to toggle out_clk.
            // Counts completed full clock periods until we hit our target.
            // -----------------------------------------------------------------
            ST_RUNNING: begin
                // Check if we have reached the end of a single clock half-period duration
                if (clk_half_period_ctr >= (CLK_DIVIDER / 2) - 1) begin
                    clk_half_period_ctr <= 0; // Reset the half-period divider counter
                    
                    if (out_clk == 1'b1) begin
                        // If the clock is currently high, this transition drops it low.
                        // A falling edge completes 1 full SPI clock cycle.
                        out_clk <= 1'b0; 
                        
                        // Check if this was the final cycle (index 4095 means 4096 total cycles)
                        if (spi_cycle_ctr == 12'hFFF) begin
                            out_latch   <= 1'b1; // Assert latch immediately at the end of the final clock period
                            latch_phase <= 1'b0; // Prepare for first half of the latch timing
                            state       <= ST_LATCH; 
                        end else begin
                            spi_cycle_ctr <= spi_cycle_ctr + 1'b1; // Increment completed cycle count
                        end
                    end else begin
                        // If the clock is currently low, toggle it high.
                        // This forms the rising edge of the SPI clock.
                        out_clk <= 1'b1; 
                    end
                end else begin
                    // Keep counting master clock cycles until the half-period is full
                    clk_half_period_ctr <= clk_half_period_ctr + 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // ST_LATCH: Keeps out_latch asserted for exactly 1 full SPI clock
            // period (2 consecutive half-period cycles).
            // -----------------------------------------------------------------
            ST_LATCH: begin
                out_clk <= 1'b0; // Ensure clock remains locked low

                if (clk_half_period_ctr >= (CLK_DIVIDER / 2) - 1) begin
                    clk_half_period_ctr <= 0; // Reset the division counter
                    
                    if (latch_phase == 1'b1) begin
                        // Second half-period timing is done. Full SPI clock period achieved.
                        out_latch <= 1'b0; // Turn off latch signal
                        state     <= ST_DONE;
                    end else begin
                        // First half-period timing is done. Proceed to second half.
                        latch_phase <= 1'b1;
                    end
                end else begin
                    clk_half_period_ctr <= clk_half_period_ctr + 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // ST_DONE: Infinite park state. Ensures lines stay locked forever.
            // -----------------------------------------------------------------
            ST_DONE: begin
                out_latch <= 1'b0; // Safely lock down latch pin low
                out_sig1  <= 1'b0; // Keep signal line 1 low permanently
                out_sig2  <= 1'b0; // Keep signal line 2 low permanently
                out_clk <= 1'b0; // Ensure clock remains locked low
                
                // No state transition out of here. The system sits here until 
                // the FPGA board is power-cycled or reconfigured.
            end
            
            default: state <= ST_IDLE;
        endcase
    end

    // Continuous drive assignments tying register values to physical BGA pins
    assign spi_sig1  = out_sig1;
    assign spi_sig2  = out_sig2;
    assign spi_clk   = out_clk;
    assign spi_latch = out_latch;

endmodule