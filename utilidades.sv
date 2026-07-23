// =============================================================
// utilidades.sv
// Infraestrutura auxiliar para debug das atividades do LASD - UFCG
// Projetado para visualizar os sinais de um processador RISC-V de ciclo unico rodando na placa Altera DE2.
// Duas interfaces de monitoramento. LCD 16x2 da placa e link UART em conjunto com um scrip python no PC
// Rafael Lima - https://github.com/rafaelbclima
//
// Contem:
//	 *Interface UART
//   - uart_tx              	: transmissor UART simples (8N1)
//   - uart_tx_test				: teste minimo para o módulo uart_tx. Envio de uma palabra de teste de 8bits pela UART
//   - rising_edge_detector  	: detector de borda do clock lento do CPU
//   - debug_dump_ctrl       	: FSM que captura o estado apos cada ciclo do CPU e envia via UART automaticamente
// 
//  *Interface LCD da placa DE2
//   - LCD_Controller			: controlador do LCD 16x2 da placa DE2
//	  - LCD_6x2						: modulo para formatar o LCD em duas linhas e 6 colunas. Cada célula tem 2 caracteres em hexa (8'hXX)
//
// Protocolo do pacote (todos os campos multi-byte big-endian, MSB primeiro):
//   [0]        HEADER      = 8'hAA  (byte de sincronismo)
//   [1:4]      PC          (32 bits)
//   [5:8]      INSTRUCTION (32 bits)
//   [9:12]     ALU_RESULT  (32 bits)
//   [13:140]   REGFILE     (32 registradores x0..x31 x 32 bits = 128 bytes)
//   [141:172]  PROBES      (8 sinais internos genericos x 32 bits = 32 bytes)
//   [173:174]  CONTROL     (16 bits, ver empacotamento abaixo)
//   [175]      CHECKSUM    (XOR de todos os bytes anteriores, sem o header)
//
// Total: 176 bytes por pacote.
//
// Empacotamento do CONTROL (16 bits, MSB->LSB):
//   bit 15    : RegWrite
//   bits 14:13: ImmSrc[1:0]
//   bit 12    : ULASrc
//   bits 11:9 : ULAControl[2:0]
//   bit 8     : MemWrite
//   bits 7:6  : ResultSrc[1:0]
//   bit 5     : Branch
//   bits 4:0  : reservado (sempre 0)
//
// Os 8 canais PROBES sao genericos: conectem qualquer sinal de 32 bits
// do datapath que queiram observar durante o desenvolvimento (saida do
// extensor de imediatos, endereco/dado de memoria, etc). Sinais com
// menos de 32 bits podem ser zero-extendidos ao conectar.
//
// =============================================================


// -------------------------------------------------------------
// debug_dump_ctrl

// Controlador de debug em modo STREAMING AUTOMATICO.
// Como a CPU dos alunos roda a no maximo 10 Hz (periodo >= 100 ms) e
// um pacote de 176 bytes a 115200 baud leva ~15 ms para ser
// transmitido, nao ha necessidade de travar o clock do processador:
// a cada borda de subida de clk_cpu, o estado inteiro eh capturado
// (em UM unico ciclo de clk_50, ja que regfile, probes e sinais de
// controle chegam todos em paralelo) e enviado automaticamente pela
// UART, com folga de sobra antes do proximo ciclo do CPU.
//
// clk_cpu eh um INPUT aqui (gerado pelos alunos, ex: um clock
// divider a partir do CLOCK_50 da placa). Este modulo nao mexe no
// clock do datapath, soh observa a borda de subida dele.
// -------------------------------------------------------------
module debug_dump_ctrl #(
    parameter int CLK_FREQ_HZ       = 50_000_000,
    parameter int BAUD_RATE         = 115_200,
    parameter int NUM_REGS          = 32,
    parameter int NUM_PROBES        = 8,
    parameter int IMM_SRC_WIDTH     = 2,
    parameter int ULA_CONTROL_WIDTH = 3,
    parameter int RESULT_SRC_WIDTH  = 2
) (
	 // clocks e resets
    input  logic        clk_50,  // clock 50MHz da placa, alimenta a logica de debug
    input  logic        rst_n,	// reset do debug. Nivel baixo ativo
    input  logic        clk_cpu,	// clock da CPU dos alunos (<=10Hz). Responsável por disparar os pacotes pela UART

    // interface com o datapath do processador
    input  logic [31:0] pc,
    input  logic [31:0] instruction,
    input  logic [31:0] alu_result,

    // banco de registradores completo, todas as 32 posicoes ligadas em paralelo (x31..x0)
    input  logic [31:0] debug_reg_data [NUM_REGS-1:0],

    // canais genericos para observar sinais internos do datapath. Conectem qualquer sinal de 32 bits que queiram monitorar
    input  logic [31:0] debug_probe [NUM_PROBES-1:0],

    // sinais de controle do datapath
    input  logic                          reg_write,
    input  logic [IMM_SRC_WIDTH-1:0]      imm_src,
    input  logic                          ula_src,
    input  logic [ULA_CONTROL_WIDTH-1:0]  ula_control,
    input  logic                          mem_write,
    input  logic [RESULT_SRC_WIDTH-1:0]   result_src,
    input  logic                          branch,

    // saida fisica para a UART
    output logic         uart_txd
);

    logic cpu_edge;
    rising_edge_detector u_edge_det (
        .clk        (clk_50),
        .rst_n      (rst_n),
        .sig_in     (clk_cpu),
        .edge_pulse (cpu_edge)
    );

    logic       uart_send;
    logic [7:0] uart_data;
    logic       uart_busy;
    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) u_uart_tx (
        .clk     (clk_50),
        .rst_n   (rst_n),
        .data_in (uart_data),
        .send    (uart_send),
        .tx      (uart_txd),
        .busy    (uart_busy)
    );

    // buffer do pacote: 1 header + 4 pc + 4 instr + 4 alu + 128 regs
    // + (NUM_PROBES*4) sondas + 2 controle + 1 checksum
    localparam int PROBES_OFFSET  = 13 + NUM_REGS*4;               // 141
    localparam int CONTROL_OFFSET = PROBES_OFFSET + NUM_PROBES*4;  // 173 (ocupa 2 bytes)
    localparam int PKT_BYTES      = CONTROL_OFFSET + 2 + 1;        // 176

    logic [7:0] pkt_buf [0:PKT_BYTES-1];
    logic [$clog2(PKT_BYTES)-1:0] pkt_idx;

    typedef enum logic [2:0] { S_IDLE, S_CAPTURE, S_FINALIZE, S_SEND, S_DONE } state_t;
    state_t state;

    logic [7:0] checksum_acc;

    // -----------------------------------------------------------
    // Todo o calculo de checksum eh feito combinacionalmente a partir
    // das fontes originais (nunca lendo pkt_buf de volta), para poder
    // ser somado numa unica atribuicao non-blocking por ciclo e evitar
    // o problema classico de varias non-blocking assigns no mesmo
    // sinal dentro de um for (onde soh a ultima "vale").
    // -----------------------------------------------------------
    logic [7:0] regs_checksum;
    always_comb begin
        regs_checksum = '0;
        for (int r = 0; r < NUM_REGS; r++) begin
            regs_checksum ^= debug_reg_data[r][31:24] ^ debug_reg_data[r][23:16]
                            ^ debug_reg_data[r][15:8]  ^ debug_reg_data[r][7:0];
        end
    end

    logic [7:0] probes_checksum;
    always_comb begin
        probes_checksum = '0;
        for (int p = 0; p < NUM_PROBES; p++) begin
            probes_checksum ^= debug_probe[p][31:24] ^ debug_probe[p][23:16]
                              ^ debug_probe[p][15:8]  ^ debug_probe[p][7:0];
        end
    end

    // palavra de controle empacotada, 16 bits (ver cabecalho do arquivo)
    logic [15:0] control_word;
    assign control_word = {reg_write, imm_src, ula_src, ula_control,
                            mem_write, result_src, branch, 5'b0};

    always_ff @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            pkt_idx       <= '0;
            uart_send     <= 1'b0;
            checksum_acc  <= '0;
            pkt_buf[0]    <= 8'hAA;
        end else begin
            uart_send <= 1'b0; // pulso de 1 ciclo por default

            case (state)
                // ---------------------------------------------
                S_IDLE: begin
                    if (cpu_edge) state <= S_CAPTURE;
                end

                // captura tudo em um unico ciclo: PC, instrucao, ALU,
                // os 32 registradores (chegam em paralelo), as sondas
                // e a palavra de controle. Tudo lido direto das portas
                // de entrada, nunca de pkt_buf.
                S_CAPTURE: begin
                    pkt_buf[1] <= pc[31:24];
                    pkt_buf[2] <= pc[23:16];
                    pkt_buf[3] <= pc[15:8];
                    pkt_buf[4] <= pc[7:0];
                    pkt_buf[5] <= instruction[31:24];
                    pkt_buf[6] <= instruction[23:16];
                    pkt_buf[7] <= instruction[15:8];
                    pkt_buf[8] <= instruction[7:0];
                    pkt_buf[9]  <= alu_result[31:24];
                    pkt_buf[10] <= alu_result[23:16];
                    pkt_buf[11] <= alu_result[15:8];
                    pkt_buf[12] <= alu_result[7:0];

                    // 32 registradores, todos capturados no mesmo ciclo
                    for (int r = 0; r < NUM_REGS; r++) begin
                        pkt_buf[13 + r*4 + 0] <= debug_reg_data[r][31:24];
                        pkt_buf[13 + r*4 + 1] <= debug_reg_data[r][23:16];
                        pkt_buf[13 + r*4 + 2] <= debug_reg_data[r][15:8];
                        pkt_buf[13 + r*4 + 3] <= debug_reg_data[r][7:0];
                    end

                    // 8 canais genericos de debug (sinais internos do datapath)
                    for (int p = 0; p < NUM_PROBES; p++) begin
                        pkt_buf[PROBES_OFFSET + p*4 + 0] <= debug_probe[p][31:24];
                        pkt_buf[PROBES_OFFSET + p*4 + 1] <= debug_probe[p][23:16];
                        pkt_buf[PROBES_OFFSET + p*4 + 2] <= debug_probe[p][15:8];
                        pkt_buf[PROBES_OFFSET + p*4 + 3] <= debug_probe[p][7:0];
                    end

                    pkt_buf[CONTROL_OFFSET]     <= control_word[15:8];
                    pkt_buf[CONTROL_OFFSET + 1] <= control_word[7:0];

                    checksum_acc <= pc[31:24] ^ pc[23:16] ^ pc[15:8] ^ pc[7:0]
                                    ^ instruction[31:24] ^ instruction[23:16]
                                    ^ instruction[15:8]  ^ instruction[7:0]
                                    ^ alu_result[31:24]  ^ alu_result[23:16]
                                    ^ alu_result[15:8]   ^ alu_result[7:0]
                                    ^ regs_checksum
                                    ^ probes_checksum
                                    ^ control_word[15:8] ^ control_word[7:0];

                    state <= S_FINALIZE;
                end

                // checksum_acc ja esta estavel (1 ciclo depois do ultimo XOR) -
                // agora sim eh seguro grava-lo no buffer
                S_FINALIZE: begin
                    pkt_buf[PKT_BYTES-1] <= checksum_acc;
                    pkt_idx <= '0;
                    state   <= S_SEND;
                end

                // envia byte a byte, respeitando o busy da UART
                S_SEND: begin
                    if (!uart_busy && !uart_send) begin
                        uart_data <= pkt_buf[pkt_idx];
                        uart_send <= 1'b1;
                        if (pkt_idx == PKT_BYTES-1) begin
                            state <= S_DONE;
                        end else begin
                            pkt_idx <= pkt_idx + 1'b1;
                        end
                    end
                end

                S_DONE: begin
                    if (!uart_busy) state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

// -------------------------------------------------------------
// Detector de borda de subida de um clock lento (clk_cpu, <=10Hz)
// sincronizado no dominio de clk_50. Usa 3 flip-flops para reduzir
// risco de metaestabilidade, ja que clk_cpu eh assincrono a clk_50.
// -------------------------------------------------------------
module rising_edge_detector (
    input  logic clk,       // clock rapido (clk_50)
    input  logic rst_n,
    input  logic sig_in,    // sinal lento a ser observado (clk_cpu)
    output logic edge_pulse // pulso de 1 ciclo de clk_50 na borda de subida
);
    logic sync0, sync1, sync2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0 <= 1'b0;
            sync1 <= 1'b0;
            sync2 <= 1'b0;
        end else begin
            sync0 <= sig_in;
            sync1 <= sync0;
            sync2 <= sync1;
        end
    end

    assign edge_pulse = sync1 & ~sync2;
endmodule

// -------------------------------------------------------------
// UART TX - 8 bits, sem paridade, 1 stop bit
// -------------------------------------------------------------
module uart_tx #(
    parameter int CLK_FREQ_HZ = 50_000_000,
    parameter int BAUD_RATE   = 115_200
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic [7:0] data_in,
    input  logic       send,       // pulso de 1 ciclo para iniciar envio
    output logic       tx,
    output logic       busy        // alto enquanto transmitindo
);

    localparam int CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;

    typedef enum logic [1:0] {IDLE, START, DATA, STOP} state_t;
    state_t state;

    logic [$clog2(CLKS_PER_BIT+1)-1:0] clk_count;
    logic [2:0] bit_index;
    logic [7:0] shift_reg;

    assign busy = (state != IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx        <= 1'b1;   // linha ociosa em nivel alto
            clk_count <= '0;
            bit_index <= '0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1'b1;
                    if (send) begin
                        shift_reg <= data_in;
                        state     <= START;
                        clk_count <= '0;
                    end
                end

                START: begin
                    tx <= 1'b0; // bit de start
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= '0;
                        bit_index <= '0;
                        state     <= DATA;
                    end
                end

                DATA: begin
                    tx <= shift_reg[bit_index];
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= '0;
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            state <= STOP;
                        end
                    end
                end

                STOP: begin
                    tx <= 1'b1; // bit de stop
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= '0;
                        state     <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule 

// =============================================================
// uart_tx_test
// Teste minimo: envia 1 byte fixo pela UART a cada pulso do
// clock de 1Hz. Serve para validar a fiacao/baud rate da UART
// antes de integrar com o resto do projeto.
//
// Ligacoes esperadas:
//   clk_50   -> CLOCK_50 da DE2
//   clk_1hz  -> seu clock de 1Hz (ja disponivel no seu projeto)
//   rst_n    -> um KEY da placa (os KEY da DE2 sao ativos em nivel
//               baixo, entao ligar direto no KEY ja funciona como
//               reset assincrono ativo baixo)
//   uart_txd -> pino GPIO ligado ao conversor USB-serial
//   led_tx   -> um LED (LEDR ou LEDG) so para "piscar" a cada envio,
//               util para confirmar visualmente que o modulo esta
//               disparando sem precisar nem abrir o terminal serial
//
// No PC: abra qualquer terminal serial (PuTTY, RealTerm, screen,
// minicom, ou o proprio Python com pyserial) na mesma baud rate
// (115200 8N1 por padrao aqui) e voce deve ver o byte 0xAA chegando
// uma vez por segundo.
// =============================================================

module uart_tx_test #(
    parameter int CLK_FREQ_HZ = 50_000_000,
    parameter int BAUD_RATE   = 115_200,
    parameter logic [7:0] TEST_BYTE = 8'hAA   // byte fixo enviado a cada segundo
) (
    input  logic clk_50,
    input  logic rst_n,
    input  logic clk_1hz,

    output logic uart_txd,
    output logic led_tx
);

    // -----------------------------------------------------------
    // Detector de borda do clock de 1Hz, sincronizado em clk_50.
    // Necessario porque clk_1hz nao deve ser usado como clock de
    // outro dominio direto (evita logica assincrona/gated clock).
    // -----------------------------------------------------------
    logic sync0, sync1, sync2;
    always_ff @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            sync0 <= 1'b0;
            sync1 <= 1'b0;
            sync2 <= 1'b0;
        end else begin
            sync0 <= clk_1hz;
            sync1 <= sync0;
            sync2 <= sync1;
        end
    end

    logic tick_1hz;
    assign tick_1hz = sync1 & ~sync2;  // pulso de 1 ciclo de clk_50 na borda de subida

    // -----------------------------------------------------------
    // UART TX
    // -----------------------------------------------------------
    logic uart_send, uart_busy;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_tx (
        .clk     (clk_50),
        .rst_n   (rst_n),
        .data_in (TEST_BYTE),
        .send    (uart_send),
        .tx      (uart_txd),
        .busy    (uart_busy)
    );

    // dispara o envio a cada tick de 1Hz (se a UART ja estiver livre,
    // o que sempre sera o caso aqui: 1 byte a 115200 baud leva menos
    // de 0.1ms, sobra tempo de sobra dentro de 1 segundo)
    assign uart_send = tick_1hz && !uart_busy;

    // LED pisca junto com cada envio, so para conferencia visual
    always_ff @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n)
            led_tx <= 1'b0;
        else if (tick_1hz)
            led_tx <= ~led_tx;
    end

endmodule

// =============================================================
// LCD_Controller
// Controlador do LCD 16x2 da placa DE2
// =============================================================
module LCD_Controller (	//	Host Side
						iDATA,iRS,
						iStart,oDone,
						iCLK,iRST_N,
						//	LCD Interface
						LCD_DATA,
						LCD_RW,
						LCD_EN,
						LCD_RS	);
	//	CLK
	parameter	CLK_Divide	=	16;

	//	Host Side
	input	[7:0]	iDATA;
	input	iRS,iStart;
	input	iCLK,iRST_N;
	output	reg		oDone;
	//	LCD Interface
	output	[7:0]	LCD_DATA;
	output	reg		LCD_EN;
	output			LCD_RW;
	output			LCD_RS;
	//	Internal Register
	reg		[4:0]	Cont;
	reg		[1:0]	ST;
	reg		preStart,mStart;

	/////////////////////////////////////////////
	//	Only write to LCD, bypass iRS to LCD_RS
	assign	LCD_DATA	=	iDATA; 
	assign	LCD_RW		=	1'b0;
	assign	LCD_RS		=	iRS;
	/////////////////////////////////////////////

	always@(posedge iCLK or negedge iRST_N)
	begin
		if(!iRST_N)
		begin
			oDone	<=	1'b0;
			LCD_EN	<=	1'b0;
			preStart<=	1'b0;
			mStart	<=	1'b0;
			Cont	<=	0;
			ST		<=	0;
		end
		else
		begin
			//////	Input Start Detect ///////
			preStart<=	iStart;
			if({preStart,iStart}==2'b01)
			begin
				mStart	<=	1'b1;
				oDone	<=	1'b0;
			end
			//////////////////////////////////
			if(mStart)
			begin
				case(ST)
				0:	ST	<=	1;	//	Wait Setup
				1:	begin
						LCD_EN	<=	1'b1;
						ST		<=	2;
					end
				2:	begin					
						if(Cont<CLK_Divide)
						Cont	<=	Cont+1;
						else
						ST		<=	3;
					end
				3:	begin
						LCD_EN	<=	1'b0;
						mStart	<=	1'b0;
						oDone	<=	1'b1;
						Cont	<=	0;
						ST		<=	0;
					end
				endcase
			end
		end
	end
endmodule

// =============================================================
// LCD_6x2
// Modulo para formatar o LCD em duas linhas e 6 colunas. Cada 
// célula tem 2 caracteres em hexa (8'hXX):
// |d0x0|d0x1|d0x2|d0x3|d0x4|d0x5|
//	|d1x0|d1x1|d1x2|d1x3|d1x4|d1x5|
// =============================================================
module	LCD_6x2 (	//	Host Side
					iCLK,iRST_N,
					d0x0,d0x1,d0x2,d0x3,d0x4,d0x5,d1x0,d1x1,d1x2,d1x3,d1x4,d1x5,
					//	LCD Side
					LCD_DATA,LCD_RW,LCD_EN,LCD_RS	);
	//	Host Side
	input			iCLK,iRST_N;
	// Data test
	input  [7:0] d0x0,d0x1,d0x2,d0x3,d0x4,d0x5,d1x0,d1x1,d1x2,d1x3,d1x4,d1x5;
	//	LCD Side
	output	[7:0]	LCD_DATA;
	output			LCD_RW,LCD_EN,LCD_RS;
	//	Internal Wires/Registers
	reg	[5:0]	LUT_INDEX;
	reg	[8:0]	LUT_DATA;
	reg	[5:0]	mLCD_ST;
	reg	[17:0]	mDLY;
	reg			mLCD_Start;
	reg	[7:0]	mLCD_DATA;
	reg			mLCD_RS;
	wire		mLCD_Done;

	parameter	LCD_INTIAL	=	0;
	parameter	LCD_RESTART	=	4;
	parameter	LCD_LINE1	=	5;
	parameter	LCD_CH_LINE	=	LCD_LINE1+16;
	parameter	LCD_LINE2	=	LCD_LINE1+16+1;
	parameter	LUT_SIZE	=	LCD_LINE2+16-1;

	always@(posedge iCLK or negedge iRST_N)
	begin
		if(!iRST_N)
		begin
			LUT_INDEX	<=	0;
			mLCD_ST		<=	0;
			mDLY		<=	0;
			mLCD_Start	<=	0;
			mLCD_DATA	<=	0;
			mLCD_RS		<=	0;
		end
		else
		begin
			begin
				case(mLCD_ST)
				0:	begin
						mLCD_DATA	<=	LUT_DATA[7:0];
						mLCD_RS		<=	LUT_DATA[8];
						mLCD_Start	<=	1;
						mLCD_ST		<=	1;
					end
				1:	begin
						if(mLCD_Done)
						begin
							mLCD_Start	<=	0;
							mLCD_ST		<=	2;					
						end
					end
				2:	begin
						if(mDLY<18'h3FFFE)
						mDLY	<=	mDLY+1;
						else
						begin
							mDLY	<=	0;
							mLCD_ST	<=	3;
						end
					end
				3:	begin
						if(LUT_INDEX<LUT_SIZE)
							LUT_INDEX	<=	LUT_INDEX+1;
						else
							LUT_INDEX	<=	LCD_RESTART;
						mLCD_ST	<=	0;
					end
				endcase
			end
		end
	end

	function [8:0] hex2char;
		input [3:0] h;
		hex2char = (h>9 ? 9'h137 : 9'h130) + h;	
	endfunction

	always
	begin
		case(LUT_INDEX)
		//	Initial
		LCD_INTIAL+0:	LUT_DATA	<=	9'h038;
		LCD_INTIAL+1:	LUT_DATA	<=	9'h00C;
		LCD_INTIAL+2:	LUT_DATA	<=	9'h001;
		LCD_INTIAL+3:	LUT_DATA	<=	9'h006;
		LCD_INTIAL+4:	LUT_DATA	<=	9'h080;
		//	Line 1	
		LCD_LINE1+0:	LUT_DATA	<=	hex2char(d0x0[ 7: 4]);
		LCD_LINE1+1:	LUT_DATA	<=	hex2char(d0x0[ 3: 0]);
		LCD_LINE1+2:	LUT_DATA	<=	9'h120;
		LCD_LINE1+3:	LUT_DATA	<=	hex2char(d0x1[ 7: 4]);             
		LCD_LINE1+4:	LUT_DATA	<=	hex2char(d0x1[ 3: 0]);
		LCD_LINE1+5:	LUT_DATA	<=	9'h120;
		LCD_LINE1+6:	LUT_DATA	<=	hex2char(d0x2[ 7: 4]);           
		LCD_LINE1+7:	LUT_DATA	<=	hex2char(d0x2[ 3: 0]);
		LCD_LINE1+8:	LUT_DATA	<=	9'h120;
		LCD_LINE1+9:	LUT_DATA	<=	hex2char(d0x3[ 7: 4]);          
		LCD_LINE1+10:	LUT_DATA	<=	hex2char(d0x3[ 3: 0]);
		LCD_LINE1+11:	LUT_DATA	<=	9'h120;
		LCD_LINE1+12:	LUT_DATA	<=	hex2char(d0x4[ 7: 4]);             
		LCD_LINE1+13:	LUT_DATA	<=	hex2char(d0x4[ 3: 0]);
		LCD_LINE1+14:	LUT_DATA	<=	9'h120;
		LCD_LINE1+15:	LUT_DATA	<=	hex2char(d0x5[ 3: 0]);
		//	Change Line               
		LCD_CH_LINE:	LUT_DATA	<=  9'h0C0;	                    
		//	Line 2                    
		LCD_LINE2+0:	LUT_DATA	<=	hex2char(d1x0[ 7: 4]);
		LCD_LINE2+1:	LUT_DATA	<=	hex2char(d1x0[ 3: 0]);
		LCD_LINE2+2:	LUT_DATA	<=	9'h120;
		LCD_LINE2+3:	LUT_DATA	<=	hex2char(d1x1[ 7: 4]);             
		LCD_LINE2+4:	LUT_DATA	<=	hex2char(d1x1[ 3: 0]);
		LCD_LINE2+5:	LUT_DATA	<=	9'h120;
		LCD_LINE2+6:	LUT_DATA	<=	hex2char(d1x2[ 7: 4]);           
		LCD_LINE2+7:	LUT_DATA	<=	hex2char(d1x2[ 3: 0]);
		LCD_LINE2+8:	LUT_DATA	<=	9'h120;
		LCD_LINE2+9:	LUT_DATA	<=	hex2char(d1x3[ 7: 4]);          
		LCD_LINE2+10:	LUT_DATA	<=	hex2char(d1x3[ 3: 0]);
		LCD_LINE2+11:	LUT_DATA	<=	9'h120;
		LCD_LINE2+12:	LUT_DATA	<=	hex2char(d1x4[ 7: 4]);             
		LCD_LINE2+13:	LUT_DATA	<=	hex2char(d1x4[ 3: 0]);
		LCD_LINE2+14:	LUT_DATA	<=	9'h120;
		LCD_LINE2+15:	LUT_DATA	<=	hex2char(d1x5[ 3: 0]);
		default:	    LUT_DATA	<=	9'h120;
		endcase
	end

	LCD_Controller 		u0	(	//	Host Side
								.iDATA(mLCD_DATA),
								.iRS(mLCD_RS),
								.iStart(mLCD_Start),
								.oDone(mLCD_Done),
								.iCLK(iCLK),
								.iRST_N(iRST_N),
								//	LCD Interface
								.LCD_DATA(LCD_DATA),
								.LCD_RW(LCD_RW),
								.LCD_EN(LCD_EN),
								.LCD_RS(LCD_RS)	);
endmodule

