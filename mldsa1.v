module qf_mldsa #(
    parameter TIMEOUT_CYCLES = 10
)(
    input logic clk,
    input logic rst_n,
    input logic mldsa_start,
    input logic [1:0] param_set,
    input logic [1:0] op_mode,
    output logic mldsa_ready,
    output logic mldsa_busy,
    output logic mldsa_done,
    output logic [15:0] b2a_error_code,
    output logic shared_req,
    input logic shared_gnt,
    input logic shared_done,
    input logic abort,
    input logic zeroize,
    output logic zeroize_req,
    input logic mask_en,
    input logic [31:0] mask_data_in
);

    typedef enum logic [2:0] {
        IDLE,
        DECODE_CFG,
        WAIT_GRANT,
        RUN_SHARED,
        FINISH,
        ZEROIZE,
        ERROR_STATE
    } state_t;

    state_t current_state, next_state;
    logic [15:0] cycle_count;
    logic [15:0] error_code_reg;
    logic [15:0] next_error_code;

    localparam logic [15:0] ERR_BAD_CONFIG = 16'h8001;
    localparam logic [15:0] ERR_TIMEOUT = 16'h8002;
    localparam logic [15:0] ERR_ABORT = 16'h8003;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            cycle_count <= 16'd0;
            error_code_reg <= 16'd0;
        end else begin
            current_state <= next_state;
            error_code_reg <= next_error_code;
            if (current_state == WAIT_GRANT ||
                current_state == RUN_SHARED) begin
                if (cycle_count < TIMEOUT_CYCLES)
                    cycle_count <= cycle_count + 1'b1;
            end else begin
                cycle_count <= 16'd0;
            end
        end
    end

    always_comb begin
        next_state = current_state;
        next_error_code = error_code_reg;
        mldsa_ready = 1'b0;
        mldsa_busy = 1'b1;
        mldsa_done = 1'b0;
        b2a_error_code = 16'd0;
        shared_req = 1'b0;
        zeroize_req = 1'b0;

        if (zeroize) begin
            next_state = ZEROIZE;
            next_error_code = ERR_ABORT;
        end else if (abort &&
                     current_state != IDLE &&
                     current_state != ZEROIZE) begin
            next_state = ZEROIZE;
            next_error_code = ERR_ABORT;
        end else begin
            case (current_state)
                IDLE: begin
                    mldsa_ready = 1'b1;
                    mldsa_busy = 1'b0;
                    next_error_code = 16'd0;
                    if (mldsa_start)
                        next_state = DECODE_CFG;
                end

                DECODE_CFG: begin
                    if (param_set > 2'b10 ||
                        op_mode > 2'b10) begin
                        next_error_code = ERR_BAD_CONFIG;
                        next_state = ERROR_STATE;
                    end else begin
                        next_state = WAIT_GRANT;
                    end
                end

                WAIT_GRANT: begin
                    shared_req = 1'b1;
                    if (shared_gnt)
                        next_state = RUN_SHARED;
                    else if (cycle_count >= TIMEOUT_CYCLES - 1) begin
                        next_error_code = ERR_TIMEOUT;
                        next_state = ERROR_STATE;
                    end
                end

                RUN_SHARED: begin
                    if (shared_done)
                        next_state = FINISH;
                    else if (cycle_count >= TIMEOUT_CYCLES - 1) begin
                        next_error_code = ERR_TIMEOUT;
                        next_state = ERROR_STATE;
                    end
                end

                FINISH: begin
                    mldsa_done = 1'b1;
                    next_state = IDLE;
                end

                ZEROIZE: begin
                    zeroize_req = 1'b1;
                    next_state = ERROR_STATE;
                end

                ERROR_STATE: begin
                    b2a_error_code = error_code_reg;
                    next_state = IDLE;
                end

                default: begin
                    next_error_code = ERR_BAD_CONFIG;
                    next_state = ERROR_STATE;
                end
            endcase
        end
    end

endmodule
