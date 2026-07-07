// =============================================================================
// led_blink/top.v — Custom board LED functionality test
//
// Cycles through 4 external LEDs (D0-D3) sequentially at ~0.52 s each,
// confirming every LED and its driver circuit is working.
//
// Clock: 16 MHz oscillator (A7)
// LEDs:  active-high, Bank 1 LVCMOS33
//   D1 → P16    D2 → M13    D0 → R16    D3 → N14
// =============================================================================
module top (
    input  wire       clk,   // 16 MHz
    output wire [3:0] led    // {D3, D0, D2, D1} → {N14, R16, M13, P16}
);

// 25-bit counter → bits [24:23] give 4 states, each lasting 2^23/16MHz ≈ 0.52 s
reg [24:0] cnt;

always @(posedge clk)
    cnt <= cnt + 25'd1;

// One LED on at a time — sequential scan
reg [3:0] led_r;
always @(*) begin
    case (cnt[23:21])
        4'd0: led_r = 4'b0001; // D1 on
        4'd1: led_r = 4'b0010; // D2 on
        4'd2: led_r = 4'b0100; // D0 on
        4'd3: led_r = 4'b1000; // D3 on
        4'd7: led_r = 4'b0001; // D1 on
        4'd6: led_r = 4'b0010; // D2 on
        4'd5: led_r = 4'b0100; // D0 on
        4'd4: led_r = 4'b1000; // D3 on
    endcase
end

assign led = led_r;

endmodule
