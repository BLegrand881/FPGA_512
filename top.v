module top (
    input   wire        clk_x1,    // 12MHz hardware clock from FTDI (Pin A10)
    input   wire        rstn,      // SW1 pushbutton (Pin P4)
    output  wire  [7:0] LED        // Output to LEDs D2-D9
);

    wire rst;          
    wire clk24M;       
    wire pll_locked;   

    assign rst = ~rstn;

    // 1. 32 MHz Clock Division Macro 
    pll_32mhz pll_inst (
        .clk_in(clk_x1),
        .reset(rst),
        .clk_out(clk24M),      
        .locked(pll_locked)    
    );

    // 2. Logic Counter running on a true 24 MHz clock
    reg [24:0] test_counter;
    
    always @(posedge clk24M or posedge rst) begin
        if (rst) begin 
            test_counter <= 25'b0;
        end else if (pll_locked) begin 
            test_counter <= test_counter + 1'b1;
        end
    end

    // Math: 32,000,000 Hz / 2^25 = ~0.71 Hz (Blinks slowly, about once per second)
    assign LED = {8{~test_counter[24]}};

endmodule


module pll_32mhz (
    input  wire clk_in,   
    input  wire reset,    
    output wire clk_out,  
    output wire locked    
);
    wire vcc = 1'b1;
    wire gnd = 1'b0;
    wire clk_fb;

    EHXPLLL #(
        .CLKI_DIV(3),          
        .CLKFB_DIV(8),        // 12MHz / 3 * 8 = 32MHz
        .CLKOP_DIV(1),        
        .CLKOP_ENABLE("ENABLED"),
        .CLKOS_ENABLE("DISABLED"),
        .CLKOS2_ENABLE("DISABLED"),
        .CLKOS3_ENABLE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"), // <--- CRITICAL: Directs hardware to use CLKOP_DIV
        .FEEDBK_PATH("CLKOP"), 
        .PLLRST_ENA("ENABLED")
    ) pll_macro (
        .CLKI(clk_in),
        .CLKFB(clk_fb),
        .PHASESEL1(gnd), .PHASESEL0(gnd), .PHASEDIR(gnd), .PHASESTEP(gnd),
        .PHASELOADREG(gnd), .STDBY(gnd),
        .RST(reset),          
        .ENCLKOP(vcc), .ENCLKOS(gnd), .ENCLKOS2(gnd), .ENCLKOS3(gnd),
        .CLKOP(clk_fb),   
        .LOCK(locked)
    );

    assign clk_out = clk_fb;
endmodule