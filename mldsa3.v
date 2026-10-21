module qf_mldsa #(
    parameter integer TIMEOUT_CYCLES = 1000
)(
    input  logic         clk,
    input  logic         rst_n,
    input  logic         mldsa_start,
    input  logic [5:0]   a2b_cmd_opcode,
    input  logic [3:0]   a2b_profile_id,
    input  logic         a2b_zeroize_req,
    input  logic         a2b_abort_req,
    output logic         mldsa_busy,
    output logic         mldsa_done,
    output logic [15:0]  b2a_error_code,
    output logic         shared_req,
    input  logic         shared_gnt,
    input  logic         shared_done,
    output logic         shared_zeroize_req,
    input  logic         shared_zeroize_ack,
    input  logic         mask_en,
    input  logic [31:0]  mask_data_in,
    output logic [31:0]  mask_share_out
);

    // Self-contained error definitions (matching qf_common_pkg)
    localparam logic [15:0] ERR_NONE                 = 16'h0000;
    localparam logic [15:0] ERR_INVALID_COMMAND     = 16'h1002;
    localparam logic [15:0] ERR_UNSUPPORTED_PROFILE = 16'h4001;
    localparam logic [15:0] ERR_TIMEOUT_CODE        = 16'h4002;
    localparam logic [15:0] ERR_ABORT_CODE          = 16'h4003;
    localparam logic [15:0] STAT_ZEROIZE_DONE       = 16'hA002;

    localparam logic [5:0] OP_KEYGEN = 6'h04;
    localparam logic [5:0] OP_SIGN   = 6'h05;
    localparam logic [5:0] OP_VERIFY = 6'h06;

    localparam logic [3:0] PROFILE_MLDSA_44 = 4'h3;
    localparam logic [3:0] PROFILE_MLDSA_65 = 4'h4;
    localparam logic [3:0] PROFILE_MLDSA_87 = 4'h5;

    localparam logic [31:0] TIMEOUT_LIMIT = TIMEOUT_CYCLES - 1;

    typedef enum logic [2:0] {
        IDLE        = 3'b000,
        DECODE_CFG  = 3'b001,
        WAIT_GRANT  = 3'b010,
        RUN_SHARED  = 3'b011,
        ZEROIZE_SEQ = 3'b100,
        FINISH      = 3'b101,
        ERROR_STATE = 3'b110
    } state_t;

    state_t current_state;
    state_t next_state;

    logic [31:0] cycle_count;
    logic [15:0] error_code_reg;
    logic [15:0] pending_error_code;
    logic [15:0] zeroize_report_code;
    logic [31:0] mask_holding_reg;

    logic profile_valid;
    logic opcode_valid;
    logic timeout_reached;
    logic operation_active;
    logic zeroize_req_seen;
    logic zeroize_request_new;
    logic abort_latched;
    logic explicit_zeroize_latched;
    logic zeroize_ack_armed;
    logic abort_event;

    always_comb begin
        profile_valid = 1'b0;

        case (a2b_profile_id)
            PROFILE_MLDSA_44,
            PROFILE_MLDSA_65,
            PROFILE_MLDSA_87:
                profile_valid = 1'b1;
            default:
                profile_valid = 1'b0;
        endcase

        opcode_valid =
            (a2b_cmd_opcode == OP_KEYGEN) ||
            (a2b_cmd_opcode == OP_SIGN)   ||
            (a2b_cmd_opcode == OP_VERIFY);

        timeout_reached     = (cycle_count >= TIMEOUT_LIMIT);
        zeroize_request_new = a2b_zeroize_req && !zeroize_req_seen;

        abort_event = a2b_abort_req &&
                      operation_active &&
                      !a2b_zeroize_req &&
                      !abort_latched &&
                      !explicit_zeroize_latched &&
                      current_state != ZEROIZE_SEQ;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state            <= IDLE;
            cycle_count              <= 32'd0;
            error_code_reg           <= ERR_NONE;
            pending_error_code       <= ERR_NONE;
            zeroize_report_code      <= STAT_ZEROIZE_DONE;
            mask_holding_reg         <= 32'd0;
            operation_active         <= 1'b0;
            zeroize_req_seen         <= 1'b0;
            abort_latched            <= 1'b0;
            explicit_zeroize_latched <= 1'b0;
            zeroize_ack_armed        <= 1'b0;
        end else begin
            current_state <= next_state;

            if (!a2b_zeroize_req)
                zeroize_req_seen <= 1'b0;
            else
                zeroize_req_seen <= 1'b1;

            if (current_state != next_state)
                cycle_count <= 32'd0;
            else if (current_state == WAIT_GRANT ||
                     current_state == RUN_SHARED) begin
                if (cycle_count < 32'hFFFF_FFFF)
                    cycle_count <= cycle_count + 1'b1;
            end else begin
                cycle_count <= 32'd0;
            end

            if (current_state != ZEROIZE_SEQ)
                zeroize_ack_armed <= 1'b0;
            else if (!shared_zeroize_ack)
                zeroize_ack_armed <= 1'b1;
            else if (zeroize_ack_armed)
                zeroize_ack_armed <= 1'b0;

            if (a2b_zeroize_req || current_state == ZEROIZE_SEQ)
                mask_holding_reg <= 32'd0;
            else if (mask_en)
                mask_holding_reg <= mask_data_in;

            if (current_state == IDLE && mldsa_start &&
                !a2b_zeroize_req && !a2b_abort_req) begin
                error_code_reg           <= ERR_NONE;
                pending_error_code       <= ERR_NONE;
                operation_active         <= 1'b1;
                abort_latched            <= 1'b0;
                explicit_zeroize_latched <= 1'b0;
                zeroize_report_code      <= STAT_ZEROIZE_DONE;
            end

            if (current_state == DECODE_CFG) begin
                if (!profile_valid)
                    pending_error_code <= ERR_UNSUPPORTED_PROFILE;
                else if (!opcode_valid)
                    pending_error_code <= ERR_INVALID_COMMAND;
            end

            if ((current_state == WAIT_GRANT ||
                 current_state == RUN_SHARED) &&
                timeout_reached &&
                !a2b_zeroize_req &&
                !a2b_abort_req &&
                !(current_state == WAIT_GRANT && shared_gnt) &&
                !(current_state == RUN_SHARED && shared_done)) begin
                pending_error_code <= ERR_TIMEOUT_CODE;
            end

            if (zeroize_request_new) begin
                explicit_zeroize_latched <= 1'b1;
                abort_latched <= 1'b0;
                zeroize_report_code <= STAT_ZEROIZE_DONE;
            end else if (abort_event) begin
                abort_latched <= 1'b1;
                zeroize_report_code <= ERR_ABORT_CODE;
            end

            if (current_state == ERROR_STATE) begin
                error_code_reg   <= pending_error_code;
                operation_active <= 1'b0;
            end

            if (current_state == FINISH)
                operation_active <= 1'b0;

            if (current_state == ZEROIZE_SEQ &&
                zeroize_ack_armed && shared_zeroize_ack) begin
                if (explicit_zeroize_latched)
                    error_code_reg <= STAT_ZEROIZE_DONE;
                else if (abort_latched)
                    error_code_reg <= ERR_ABORT_CODE;
                else
                    error_code_reg <= zeroize_report_code;

                operation_active         <= 1'b0;
                abort_latched            <= 1'b0;
                explicit_zeroize_latched <= 1'b0;
            end
        end
    end

    always_comb begin
        next_state = current_state;

        mldsa_busy = (current_state != IDLE) ||
                     zeroize_request_new ||
                     (abort_event);

        mldsa_done         = 1'b0;
        shared_req         = 1'b0;
        shared_zeroize_req = 1'b0;
        b2a_error_code     = error_code_reg;
        mask_share_out     = mask_holding_reg;

        if (current_state == ZEROIZE_SEQ) begin
            shared_zeroize_req = 1'b1;

            if (zeroize_ack_armed && shared_zeroize_ack)
                next_state = IDLE;
        end else if (zeroize_request_new) begin
            next_state = ZEROIZE_SEQ;
        end else if (abort_event) begin
            next_state = ZEROIZE_SEQ;
        end else begin
            case (current_state)
                IDLE: begin
                    if (mldsa_start &&
                        !a2b_zeroize_req &&
                        !a2b_abort_req)
                        next_state = DECODE_CFG;
                end

                DECODE_CFG: begin
                    if (!profile_valid || !opcode_valid)
                        next_state = ERROR_STATE;
                    else
                        next_state = WAIT_GRANT;
                end

                WAIT_GRANT: begin
                    shared_req = 1'b1;

                    if (shared_gnt)
                        next_state = RUN_SHARED;
                    else if (timeout_reached)
                        next_state = ERROR_STATE;
                end

                RUN_SHARED: begin
                    shared_req = 1'b1;

                    if (shared_done)
                        next_state = FINISH;
                    else if (timeout_reached)
                        next_state = ERROR_STATE;
                end

                FINISH: begin
                    mldsa_done = 1'b1;
                    next_state = IDLE;
                end

                ERROR_STATE: begin
                    next_state = IDLE;
                end

                default: begin
                    next_state = IDLE;
                end
            endcase
        end
    end

    initial begin
        if (TIMEOUT_CYCLES < 1)
            $error("TIMEOUT_CYCLES must be >= 1");
    end

endmodule