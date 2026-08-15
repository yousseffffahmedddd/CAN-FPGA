
# vlib work
# vmap work work

# echo "Compiling RTL files..."
# vlog -sv rtl/top/can_top.v
# vlog -sv rtl/control_logic/control_logic.v
# vlog -sv rtl/spi_if/spi_if.v
# vlog -sv rtl/protocol_engine/protocol_engine.v
# vlog -sv rtl/protocol_engine/fsm_part1.v
# vlog -sv rtl/protocol_engine/fsm_part2.v
# vlog -sv rtl/protocol_engine/fsm_part3.v
# vlog -sv rtl/protocol_engine/bit_stuffer.v
# vlog -sv rtl/protocol_engine/crc_gen_check.v
# vlog -sv rtl/protocol_engine/error_mgmt.v
# vlog -sv rtl/ctrl_int_regs/reg_bank.v
# vlog -sv rtl/ctrl_int_regs/irq_ctrl.v
# vlog -sv rtl/ctrl_int_regs/mode_fsm.v
# vlog -sv rtl/buffers_filters/tx_buffer.v
# vlog -sv rtl/buffers_filters/rx_buffer.v
# vlog -sv rtl/buffers_filters/accept_filter.v
# vlog -sv rtl/common/*.v


# echo "Compiling testbench..."
# # vlog -sv tb/spi_if/tb_spi_if.v
# # vlog -sv tb/tb_can_top_complete.v
# vlog -sv tb/tb_endToendTest.v
# echo "Starting simulation..."
# # vsim -c work.tb_spi_if
# # vsim -c work.tb_can_top_complete
# vsim -c  work.tb_spi_can_e2e

# run -all
# quit -f
# =====================================================================
# ModelSim / QuestaSim Waveform Configuration Script
# =====================================================================

# =====================================================================
# ModelSim / QuestaSim Quick Run & Waveform TCL Script
# =====================================================================
# =====================================================================
# ModelSim / QuestaSim Complete Run & Waveform TCL Script
# =====================================================================

# =====================================================================
# # ModelSim / QuestaSim Complete Run & Waveform TCL Script
# # =====================================================================

# 1. Reset work library
if {[file exists work]} {
    vdel -lib work -all
}
vlib work

# 2. Compile Common Components & Headers
vlog -work work "rtl/common/can_defs.vh"
vlog -work work "rtl/common/bit_clk_gen.v"
vlog -work work "rtl/common/cdc_handshake.v"
vlog -work work "rtl/common/flag_reg.v"
vlog -work work "rtl/common/rxcan_sync.v"
vlog -work work "rtl/common/sat_counter.v"
vlog -work work "rtl/common/shift_reg.v"
vlog -work work "rtl/common/sync_2ff_edge.v"

# 3. Compile Control, Registers, and SPI Interface
vlog -work work "rtl/ctrl_int_regs/irq_ctrl.v"
vlog -work work "rtl/ctrl_int_regs/mode_fsm.v"
vlog -work work "rtl/ctrl_int_regs/reg_bank.v"
vlog -work work "rtl/spi_if/spi_if.v"

# 4. Compile Buffers and Filters (Required for can_top)
vlog -work work "rtl/buffers_filters/tx_buffer.v"
vlog -work work "rtl/buffers_filters/accept_filter.v"
vlog -work work "rtl/buffers_filters/rx_buffer.v"

# 5. Compile Protocol Engine Sub-blocks
vlog -work work "rtl/protocol_engine/bit_stuffer.v"
vlog -work work "rtl/protocol_engine/crc_gen_check.v"
vlog -work work "rtl/protocol_engine/error_mgmt.v"
vlog -work work "rtl/protocol_engine/fsm_part1.v"
vlog -work work "rtl/protocol_engine/fsm_part2.v"
vlog -work work "rtl/protocol_engine/fsm_part3.v"
vlog -work work "rtl/protocol_engine/protocol_engine.v"

# 6. Compile Top Wrapper and Testbench
vlog -work work "rtl/top/can_top.v"
vlog -work work "tb/tb_endToendTest.v"

# 7. Start Simulation (with acceleration)
vsim -voptargs="+acc" work.tb_spi_can_e2e

# 8. Set Up Waveform Window
view wave
delete wave *

add wave -noupdate -divider {Testbench & SPI Bus}
add wave -noupdate -color Yellow /tb_spi_can_e2e/clk
add wave -noupdate -color Yellow /tb_spi_can_e2e/rst_n
add wave -noupdate /tb_spi_can_e2e/sck
add wave -noupdate /tb_spi_can_e2e/cs_n
add wave -noupdate /tb_spi_can_e2e/si
add wave -noupdate /tb_spi_can_e2e/so

add wave -noupdate -divider {CAN Bus & Top Wrapper}
add wave -noupdate -color Orange /tb_spi_can_e2e/rx_pin
add wave -noupdate -color Orange /tb_spi_can_e2e/tx_can
add wave -noupdate /tb_spi_can_e2e/tx_en
add wave -noupdate /tb_spi_can_e2e/int_n

add wave -noupdate -divider {Protocol FSM & Verification}
add wave -noupdate -radix hexadecimal /tb_spi_can_e2e/u_can_top/u_protocol_engine/current_state
add wave -noupdate /tb_spi_can_e2e/u_can_top/u_protocol_engine/bit_tick
add wave -noupdate -radix unsigned /tb_spi_can_e2e/bit_idx
add wave -noupdate -radix hexadecimal /tb_spi_can_e2e/captured_bits

# 9. Run Simulation
run -all
wave zoom full