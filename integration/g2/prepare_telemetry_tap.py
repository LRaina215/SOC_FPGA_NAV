"""Export only the UART's already accepted bytes in the generated control copy."""
from pathlib import Path


def replace_once(text, old, new):
    assert text.count(old) == 1, ('unexpected frozen source', old)
    return text.replace(old, new)


def prepare(out):
    out = Path(out)
    uart = out/'control/uart_telemetry.v'
    text = uart.read_text(encoding='utf-8')
    text = replace_once(text, '    output wire tx\n',
        '    output wire tx,\n    output wire tap_valid,tap_first,tap_last,\n    output wire [7:0] tap_data\n')
    text = replace_once(text, '    wire ready;\n', '    wire ready;\n'
        '    assign tap_valid=active && ready && reset_n;\n'
        '    assign tap_first=(index==0);\n'
        '    assign tap_last=(index==PAYLOAD_BYTES+5);\n'
        '    assign tap_data=data;\n')
    uart.write_text(text, encoding='utf-8', newline='\n')
    core = out/'control4500_core.v'
    text = core.read_text(encoding='utf-8')
    text = replace_once(text, '    output wire feedback_valid,\n',
        '    output wire feedback_valid,\n'
        '    output wire telemetry_valid,telemetry_first,telemetry_last,\n'
        '    output wire [7:0] telemetry_data,\n')
    text = replace_once(text, '.payload(telemetry_payload),.tx(UART_TXD)',
        '.payload(telemetry_payload),.tx(UART_TXD),\n'
        '        .tap_valid(telemetry_valid),.tap_first(telemetry_first),\n'
        '        .tap_last(telemetry_last),.tap_data(telemetry_data)')
    core.write_text(text, encoding='utf-8', newline='\n')
