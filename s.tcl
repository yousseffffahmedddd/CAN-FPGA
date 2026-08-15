
# # vlib work
# # vmap work work

# # echo "Compiling RTL files..."
# # vlog -sv rtl/top/can_top.v
# # vlog -sv rtl/control_logic/control_logic.v
# # vlog -sv rtl/spi_if/spi_if.v
# # vlog -sv rtl/protocol_engine/protocol_engine.v
# # vlog -sv rtl/protocol_engine/fsm_part1.v
# # vlog -sv rtl/protocol_engine/fsm_part2.v
# # vlog -sv rtl/protocol_engine/fsm_part3.v
# # vlog -sv rtl/protocol_engine/bit_stuffer.v
# # vlog -sv rtl/protocol_engine/crc_gen_check.v
# # vlog -sv rtl/protocol_engine/error_mgmt.v
# # vlog -sv rtl/ctrl_int_regs/reg_bank.v
# # vlog -sv rtl/ctrl_int_regs/irq_ctrl.v
# # vlog -sv rtl/ctrl_int_regs/mode_fsm.v
# # vlog -sv rtl/buffers_filters/tx_buffer.v
# # vlog -sv rtl/buffers_filters/rx_buffer.v
# # vlog -sv rtl/buffers_filters/accept_filter.v
# # vlog -sv rtl/common/*.v


# # echo "Compiling testbench..."
# # # vlog -sv tb/spi_if/tb_spi_if.v
# # # vlog -sv tb/tb_can_top_complete.v
# # vlog -sv tb/tb_endToendTest.v
# # echo "Starting simulation..."
# # # vsim -c work.tb_spi_if
# # # vsim -c work.tb_can_top_complete
# # vsim -c  work.tb_spi_can_e2e

# # run -all
# # quit -f
# # =====================================================================
# # ModelSim / QuestaSim Waveform Configuration Script
# # =====================================================================

# # =====================================================================
# # ModelSim / QuestaSim Quick Run & Waveform TCL Script
# # =====================================================================
# # =====================================================================
# # ModelSim / QuestaSim Complete Run & Waveform TCL Script
# # =====================================================================

# # =====================================================================
# # # ModelSim / QuestaSim Complete Run & Waveform TCL Script
# # # =====================================================================

# # Always run relative paths from the directory that contains this script.
# # This lets the same TCL work whether it is launched from CAN-FPGA or from
# # can-controller-fpga.
# set SCRIPT_DIR [file dirname [file normalize [info script]]]
# cd $SCRIPT_DIR

# # 1. Reset work library
# if {[file exists work]} {
#     vdel -lib work -all
# }
# vlib work

# # 2. Compile Common Components & Headers
# vlog -work work "rtl/common/can_defs.vh"
# vlog -work work "rtl/common/bit_clk_gen.v"
# vlog -work work "rtl/common/cdc_handshake.v"
# vlog -work work "rtl/common/flag_reg.v"
# vlog -work work "rtl/common/rxcan_sync.v"
# vlog -work work "rtl/common/sat_counter.v"
# vlog -work work "rtl/common/shift_reg.v"
# vlog -work work "rtl/common/sync_2ff_edge.v"

# # 3. Compile Control Logic, Registers, and SPI Interface
# # control_logic is now instantiated by can_top and must be present in work.
# vlog -work work "rtl/control_logic/control_logic.v"
# # irq_ctrl.v and mode_fsm.v are legacy optional helpers and are not instantiated
# # by the reduced Normal-mode top level, so the main E2E flow does not compile
# # them as active architecture. Their source files are left untouched.
# vlog -work work "rtl/ctrl_int_regs/reg_bank.v"
# vlog -work work "rtl/spi_if/spi_if.v"

# # 4. Compile Buffers and Filters (Required for can_top)
# vlog -work work "rtl/buffers_filters/tx_buffer.v"
# vlog -work work "rtl/buffers_filters/accept_filter.v"
# vlog -work work "rtl/buffers_filters/rx_buffer.v"

# # 5. Compile Protocol Engine Sub-blocks
# vlog -work work "rtl/protocol_engine/bit_stuffer.v"
# vlog -work work "rtl/protocol_engine/crc_gen_check.v"
# vlog -work work "rtl/protocol_engine/error_mgmt.v"
# vlog -work work "rtl/protocol_engine/fsm_part1.v"
# vlog -work work "rtl/protocol_engine/fsm_part2.v"
# vlog -work work "rtl/protocol_engine/fsm_part3.v"
# vlog -work work "rtl/protocol_engine/protocol_engine.v"

# # 6. Compile Top Wrapper and Testbench
# vlog -work work "rtl/top/can_top.v"
# vlog -work work "tb/tb_endToendTest.v"

# # 7. Start Simulation (with acceleration)
# vsim -voptargs="+acc" work.tb_spi_can_e2e

# # 8. Console-friendly signal logging
# # Keep the run usable with `vsim -c`. Questa writes the simulation database
# # without requiring a GUI waveform window.
# log -r /*

# # 9. Run Simulation
# run -all


# =====================================================================
# ModelSim / QuestaSim Interactive Waveform TCL Script for CAN E2E Test
# =====================================================================

# Always run relative paths from the directory that contains this script.
set SCRIPT_DIR [file dirname [file normalize [info script]]]
cd $SCRIPT_DIR

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

# 3. Compile Control Logic, Registers, and SPI Interface
vlog -work work "rtl/control_logic/control_logic.v"
vlog -work work "rtl/ctrl_int_regs/reg_bank.v"
vlog -work work "rtl/spi_if/spi_if.v"

# 4. Compile Buffers and Filters
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

# 7. Start Simulation in GUI Mode (Removed -c and ensured +acc visibility)
vsim -voptargs="+acc" work.tb_spi_can_e2e

# 8. Log all signals for waveform recording
log -r /*

# 9. Add signals to the waveform window
add wave -r /*

# 10. Open the Wave window explicitly
view wave

# 11. Run Simulation
run -all

# 12. Zoom to fit the entire waveform in the viewer window
wave zoom full