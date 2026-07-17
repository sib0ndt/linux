ccflags-y += -I$(src)/include
ccflags-$(CONFIG_COMMON_CLK_RTD1619) += -I$(src)/include/

obj-$(CONFIG_COMMON_CLK_REALTEK) += clk-rtk.o
obj-$(CONFIG_COMMON_CLK_DUMMY) += clk-dummy.o

clk-rtk-y += common.o
clk-rtk-y += clk-mmio-mux.o
clk-rtk-y += clk-mmio-gate.o
clk-rtk-y += cgc.o
clk-rtk-y += clk-pll.o
clk-rtk-y += platform.o
clk-rtk-y += reset.o
clk-rtk-$(CONFIG_RESET_MUX) += reset-mux.o
clk-rtk-$(CONFIG_COMMON_CLK_RTD1619) += rtd1619.o


