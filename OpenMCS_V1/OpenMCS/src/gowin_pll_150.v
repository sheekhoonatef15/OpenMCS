// Tang Primer 25K GW5A PLLA: 50 MHz input -> 100 MHz and 300 MHz, both 0 deg.
// The input synchronizers use the 300 MHz falling edge; the histogram captures
// on the 100 MHz rising edge, avoiding an aligned-edge tuple transfer.
// VCO = 50 MHz / 1 * 1 * 24 = 1200 MHz (within 700--1400 MHz).
// CLKOUT0 = 1200 / 12 = 100 MHz (acquisition / histogram domain)
// CLKOUT1 = 1200 /  4 = 300 MHz (input synchronizer domain)
// Both outputs derive from the same VCO with zero phase offset.
module gowin_pll_150 (
    input  wire clkin,
    output wire clkout_0,
    output wire clkout_300,
    output wire lock
);
    wire gw_gnd = 1'b0;
    wire clkout2_unused, clkout3_unused, clkout4_unused, clkout5_unused, clkout6_unused, clkfbout_unused;
    wire [7:0] mdrdo_unused;

    PLLA pll_inst (
        .LOCK(lock),
        .CLKOUT0(clkout_0), .CLKOUT1(clkout_300),
        .CLKOUT2(clkout2_unused), .CLKOUT3(clkout3_unused),
        .CLKOUT4(clkout4_unused), .CLKOUT5(clkout5_unused),
        .CLKOUT6(clkout6_unused), .CLKFBOUT(clkfbout_unused),
        .MDRDO(mdrdo_unused), .CLKIN(clkin), .CLKFB(gw_gnd),
        .RESET(gw_gnd), .PLLPWD(gw_gnd),
        .RESET_I(gw_gnd), .RESET_O(gw_gnd),
        .PSSEL(3'b000), .PSDIR(gw_gnd), .PSPULSE(gw_gnd),
        .SSCPOL(gw_gnd), .SSCON(gw_gnd), .SSCMDSEL(7'b0),
        .SSCMDSEL_FRAC(3'b0), .MDCLK(gw_gnd), .MDOPC(2'b0),
        .MDAINC(gw_gnd), .MDWDI(8'b0)
    );

    defparam pll_inst.FCLKIN = "50";
    defparam pll_inst.IDIV_SEL = 1;
    defparam pll_inst.FBDIV_SEL = 1;
    defparam pll_inst.CLKFB_SEL = "INTERNAL";
    defparam pll_inst.MDIV_SEL = 24;
    defparam pll_inst.MDIV_FRAC_SEL = 0;
    defparam pll_inst.ODIV0_SEL = 12;
    defparam pll_inst.ODIV0_FRAC_SEL = 0;
    defparam pll_inst.ODIV1_SEL = 4;
    defparam pll_inst.ODIV2_SEL = 12;
    defparam pll_inst.ODIV3_SEL = 12;
    defparam pll_inst.ODIV4_SEL = 8;
    defparam pll_inst.ODIV5_SEL = 8;
    defparam pll_inst.ODIV6_SEL = 8;
    defparam pll_inst.CLKOUT0_EN = "TRUE";
    defparam pll_inst.CLKOUT1_EN = "TRUE";
    defparam pll_inst.CLKOUT2_EN = "FALSE";
    defparam pll_inst.CLKOUT3_EN = "FALSE";
    defparam pll_inst.CLKOUT4_EN = "FALSE";
    defparam pll_inst.CLKOUT5_EN = "FALSE";
    defparam pll_inst.CLKOUT6_EN = "FALSE";
    defparam pll_inst.CLKOUT0_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT1_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT2_DT_DIR = 1'b1;
    defparam pll_inst.CLKOUT3_DT_DIR = 1'b1;
    defparam pll_inst.CLK0_IN_SEL = 1'b0;
    defparam pll_inst.CLK0_OUT_SEL = 1'b0;
    defparam pll_inst.CLK1_IN_SEL = 1'b0;
    defparam pll_inst.CLK1_OUT_SEL = 1'b0;
    defparam pll_inst.CLK2_IN_SEL = 1'b0;
    defparam pll_inst.CLK2_OUT_SEL = 1'b0;
    defparam pll_inst.CLK3_IN_SEL = 1'b0;
    defparam pll_inst.CLK3_OUT_SEL = 1'b0;
    defparam pll_inst.CLK4_IN_SEL = 2'b00;
    defparam pll_inst.CLK4_OUT_SEL = 1'b0;
    defparam pll_inst.CLK5_IN_SEL = 1'b0;
    defparam pll_inst.CLK5_OUT_SEL = 1'b0;
    defparam pll_inst.CLK6_IN_SEL = 1'b0;
    defparam pll_inst.CLK6_OUT_SEL = 1'b0;
    defparam pll_inst.CLKOUT0_PE_COARSE = 0;
    defparam pll_inst.CLKOUT0_PE_FINE = 0;
    defparam pll_inst.CLKOUT1_PE_COARSE = 0;
    defparam pll_inst.CLKOUT1_PE_FINE = 0;
    defparam pll_inst.CLKOUT2_PE_COARSE = 0;
    defparam pll_inst.CLKOUT2_PE_FINE = 0;
    defparam pll_inst.CLKOUT3_PE_COARSE = 0;
    defparam pll_inst.CLKOUT3_PE_FINE = 0;
    defparam pll_inst.CLKOUT4_PE_COARSE = 0;
    defparam pll_inst.CLKOUT4_PE_FINE = 0;
    defparam pll_inst.CLKOUT5_PE_COARSE = 0;
    defparam pll_inst.CLKOUT5_PE_FINE = 0;
    defparam pll_inst.CLKOUT6_PE_COARSE = 0;
    defparam pll_inst.CLKOUT6_PE_FINE = 0;
    defparam pll_inst.DE0_EN = "FALSE";
    defparam pll_inst.DE1_EN = "FALSE";
    defparam pll_inst.DE2_EN = "FALSE";
    defparam pll_inst.DE3_EN = "FALSE";
    defparam pll_inst.DE4_EN = "FALSE";
    defparam pll_inst.DE5_EN = "FALSE";
    defparam pll_inst.DE6_EN = "FALSE";
    defparam pll_inst.DYN_DPA_EN = "FALSE";
    defparam pll_inst.DYN_PE0_SEL = "FALSE";
    defparam pll_inst.DYN_PE1_SEL = "FALSE";
    defparam pll_inst.DYN_PE2_SEL = "FALSE";
    defparam pll_inst.DYN_PE3_SEL = "FALSE";
    defparam pll_inst.DYN_PE4_SEL = "FALSE";
    defparam pll_inst.DYN_PE5_SEL = "FALSE";
    defparam pll_inst.DYN_PE6_SEL = "FALSE";
    defparam pll_inst.RESET_I_EN = "FALSE";
    defparam pll_inst.RESET_O_EN = "FALSE";
    defparam pll_inst.ICP_SEL = 6'bXXXXXX;
    defparam pll_inst.LPF_RES = 3'bXXX;
    defparam pll_inst.LPF_CAP = 2'b00;
    defparam pll_inst.SSC_EN = "FALSE";
    defparam pll_inst.CLKOUT0_DT_STEP = 0;
    defparam pll_inst.CLKOUT1_DT_STEP = 0;
    defparam pll_inst.CLKOUT2_DT_STEP = 0;
    defparam pll_inst.CLKOUT3_DT_STEP = 0;
endmodule
