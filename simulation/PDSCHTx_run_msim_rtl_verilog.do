transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/LDPC_prepv.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_tb_parallel.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_cb_parallel.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_serial.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_update.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/PDSCHTx.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/Ceil.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/LDPC_encode.v}

vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/Ceil.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_cb_parallel.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_serial.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_update.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/crc_tb_parallel.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/LDPC_encode.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/LDPC_prepv.v}
vlog  -work work +incdir+D:/Docs/Uczelnia/Dyplom1/rtl {D:/Docs/Uczelnia/Dyplom1/rtl/PDSCHTx.v}
vlog -sv -work work +incdir+D:/Docs/Uczelnia/Dyplom1/test_bench {D:/Docs/Uczelnia/Dyplom1/test_bench/test_PDSCH.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L cyclonev_hssi_ver -L cyclonev_pcie_hip_ver -L rtl_work -L work -voptargs="+acc"  test_PDSCH

add wave *
view structure
view signals
run -all
