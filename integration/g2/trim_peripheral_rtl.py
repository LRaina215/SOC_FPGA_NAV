"""Explicit peripheral cuts; retain control, navigation, telemetry and transport.

These are feature trades, not equivalence claims for the removed diagnostics.
Each transform refuses an unexpected source body instead of guessing a cut.
"""
import re

OLED_RELATIVE = 'control4500_core.v'
TX_SHADOW_RELATIVE = 'nav/rtl/net/host_mbx.v'
CONSOLE_RELATIVE = 'nav/rtl/soc/nav_soc.v'

OLED_REPLACEMENT = '''    // Trim profile: OLED disabled; release both open-drain pins.
    assign sck = 1'bz;
    assign sda = 1'bz;'''
SHADOW_REPLACEMENT = '''    // Trim profile: offline TX mirror readback is unavailable; live u_txb remains.
    assign txb_q = 32'd0;'''
CONSOLE_REPLACEMENT = '''    // Trim profile: discard diagnostic log storage; CPU write ACK and counters remain.
    assign con_q = 32'd0;'''


def trim_oled(text):
    text = text.replace('\r\n', '\n')
    pattern = re.compile(r'    oled #\(.*?\n    \) display\(.*?\.IIC_SCL\(sck\),\.IIC_SDA\(sda\),\.ready\(\),\.error\(\)\);', re.S)
    matches = list(pattern.finditer(text))
    assert len(matches) == 1, 'expected exactly one standalone OLED instance'
    old = matches[0][0]
    assert '.clk(clk),.reset_n(device_reset_n)' in old
    changed = text[:matches[0].start()]+OLED_REPLACEMENT+text[matches[0].end():]
    assert changed.replace(OLED_REPLACEMENT, old) == text
    assert changed.count('uart_telemetry #(') == text.count('uart_telemetry #(') == 1
    return changed


def trim_tx_shadow(text):
    text = text.replace('\r\n', '\n')
    pattern = re.compile(r'    nav_ram_sdp #\(\.DW\(32\), \.AW\(TX_WAW\)\) u_txs \(.*?\n    \);', re.S)
    matches = list(pattern.finditer(text))
    assert len(matches) == 1, 'expected exactly one independent TX readback shadow'
    old = matches[0][0]
    assert '.we(tx_we & ~twp[TX_WAW])' in old
    assert '.re(txb_re), .raddr(txb_addr), .rdata(txb_q)' in old
    changed = text[:matches[0].start()]+SHADOW_REPLACEMENT+text[matches[0].end():]
    assert changed.replace(SHADOW_REPLACEMENT, old) == text
    assert ' u_txb (' in changed and ' u_injb (' in changed
    return changed


def trim_console_storage(text):
    text = text.replace('\r\n', '\n')
    marker = '    // ================================================================ 控制台环形缓冲'
    start = text.index(marker)
    begin = text.index('    generate\n', start)
    end = text.index('    endgenerate', begin)+len('    endgenerate')
    old = text[begin:end]
    assert old.count(' u_con (') == 1 and 'begin : g_con' in old
    assert '.rdata(con_q[8*b +: 8])' in old
    changed = text[:begin]+CONSOLE_REPLACEMENT+text[end:]
    assert changed.replace(CONSOLE_REPLACEMENT, old) == text
    # The serial protocol, RAM download/HOLD, CPU peripheral ACK and count remain.
    for required in ('begin : g_lane', 'assign dbg_ram_we =', "16'hF000: hold <= d_wdata[0]",
                     "if (req && sel_per) begin", "con_wp <= con_wp + 1'b1",
                     "d_rdata <= d_ram ? (hold ? ram_q : 32'd0) : con_q"):
        assert required in changed
    return changed
