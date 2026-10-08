module uart_tx_8n1 #(
    parameter integer CLOCK_HZ = 50_000_000,
    parameter integer BAUD = 5_000_000
) (
    input wire clk, input wire [7:0] data, input wire valid,
    output wire ready, output wire tx
);
    localparam integer CLKS_PER_BIT = CLOCK_HZ / BAUD;
    localparam integer COUNT_WIDTH = $clog2(CLKS_PER_BIT);
    reg busy = 1'b0;
    reg [COUNT_WIDTH-1:0] clock_count = {COUNT_WIDTH{1'b0}};
    reg [3:0] bit_index = 4'd0;
    reg [9:0] frame = 10'h3ff;
    assign ready = ~busy;
    assign tx = busy ? frame[bit_index] : 1'b1;
    always @(posedge clk) begin
        if (!busy) begin
            if (valid) begin
                frame <= {1'b1, data, 1'b0};
                bit_index <= 4'd0;
                clock_count <= CLKS_PER_BIT - 1;
                busy <= 1'b1;
            end
        end else if (clock_count == 0) begin
            clock_count <= CLKS_PER_BIT - 1;
            if (bit_index == 4'd9) busy <= 1'b0;
            else bit_index <= bit_index + 1'b1;
        end else clock_count <= clock_count - 1'b1;
    end
endmodule
