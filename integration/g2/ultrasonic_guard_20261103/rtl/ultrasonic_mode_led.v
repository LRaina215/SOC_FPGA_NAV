// Repurpose only LED7/N17, previously the high bit of the firmware LED counter.
// Existing heartbeat, IRQ, ESTOP, watchdog, CPU low3 bits and trap LEDs stay intact.
module ultrasonic_mode_led #(parameter integer CLOCK_HZ=50000000)(
    input wire clk,reset_n,enabled,sample_healthy,
    input wire [3:0] cpu_leds,
    output wire [3:0] board_cpu_leds
);
    // Own resettable phase: the original top-level hb has no RTL reset/INIT.
    reg [31:0] blink_count;
    reg blink_phase;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin blink_count<=0;blink_phase<=1;end
        else if(!enabled) begin blink_count<=0;blink_phase<=1;end
        else if(blink_count==CLOCK_HZ/4-1) begin blink_count<=0;blink_phase<=!blink_phase;end
        else blink_count<=blink_count+1'b1;
    end
    assign board_cpu_leds[2:0]=cpu_leds[2:0];
    assign board_cpu_leds[3]=reset_n && enabled && (sample_healthy || blink_phase);
endmodule
