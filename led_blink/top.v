// =============================================================================
// led_blink/top.v — ECP5-5G LED functionality test
//
// Cycles through 4 external LEDs (D0-D3) sequentially at ~0.7 s each,
// confirming every LED and its driver circuit is working.
//
// Clock: 12 MHz FTDI (A10)
// LEDs:  active-high, Bank 1 LVCMOS33
//   D3 → N14    D0 → R16    D2 → M13    D1 → P16
// =============================================================================
module top (
    input  wire       clk,   // 12 MHz
    input  wire       rstn,  // SW4, active-low
    output wire [3:0] led    // {D3, D0, D2, D1} → {N14, R16, M13, P16}
);

// 25-bit counter → bits [24:23] give 4 states, each lasting 2^23/12MHz ≈ 0.70 s
reg [24:0] cnt;

always @(posedge clk or negedge rstn) begin
    if (!rstn)
        cnt <= 25'd0;
    else
        cnt <= cnt + 25'd1;
end

// One LED on at a time — sequential scan
reg [3:0] led_r;
always @(*) begin
    case (cnt[24:23])
        2'd0: led_r = 4'b0001; // D1 on
        2'd1: led_r = 4'b0010; // D2 on
        2'd2: led_r = 4'b0100; // D0 on
        2'd3: led_r = 4'b1000; // D3 on
    endcase
end

assign led = led_r;

endmodule
