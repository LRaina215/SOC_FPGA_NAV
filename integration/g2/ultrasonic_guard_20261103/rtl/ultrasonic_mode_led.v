//LED7 directly reports the selected mode. Other CPU LED bits are unchanged.
module ultrasonic_mode_led(input wire enabled,input wire [3:0] cpu_leds,output wire [3:0] board_cpu_leds);
    assign board_cpu_leds={enabled,cpu_leds[2:0]};
endmodule
