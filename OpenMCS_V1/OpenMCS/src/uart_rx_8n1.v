module uart_rx_8n1 #(
    parameter integer CLOCK_HZ = 50_000_000,
    parameter integer BAUD = 5_000_000
) (
    input wire clk,
    input wire rx,
    output reg [7:0] data = 8'h00,
    output reg valid = 1'b0,
    output reg framing_error = 1'b0
);
    localparam integer CLKS_PER_BIT = CLOCK_HZ / BAUD;
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;
    localparam integer COUNT_WIDTH = $clog2(CLKS_PER_BIT);

    reg rx_meta = 1'b1, rx_sync = 1'b1;
    reg busy = 1'b0;
    reg [COUNT_WIDTH-1:0] clock_count = {COUNT_WIDTH{1'b0}};
    reg [3:0] bit_index = 4'd0;

    always @(posedge clk) begin
        rx_meta <= rx;
        rx_sync <= rx_meta;
        valid <= 1'b0;
        framing_error <= 1'b0;

        if (!busy) begin
            if (!rx_sync) begin
                busy <= 1'b1;
                bit_index <= 4'd0;
                clock_count <= HALF_BIT - 1;
            end
        end else if (clock_count != 0) begin
            clock_count <= clock_count - 1'b1;
        end else if (bit_index == 0) begin
            if (rx_sync) begin
                busy <= 1'b0;
                framing_error <= 1'b1;
            end else begin
                bit_index <= 4'd1;
                clock_count <= CLKS_PER_BIT - 1;
            end
        end else if (bit_index <= 8) begin
            data[bit_index - 1'b1] <= rx_sync;
            bit_index <= bit_index + 1'b1;
            clock_count <= CLKS_PER_BIT - 1;
        end else begin
            busy <= 1'b0;
            if (rx_sync) valid <= 1'b1;
            else framing_error <= 1'b1;
        end
    end
endmodule
