`default_nettype none //Comando para desabilitar declaração automática de wires
module Mod_Teste (
	//Clocks
	input CLOCK_27, CLOCK_50,
	//Chaves e Botoes
	input [3:0] KEY,
	input [17:0] SW,
	//Displays de 7 seg e LEDs
	output [0:6] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
	output [8:0] LEDG,
	output [17:0] LEDR,
	//Serial
	output UART_TXD,
	input UART_RXD,
	//LCD
	inout [7:0] LCD_DATA,
	output LCD_ON, LCD_BLON, LCD_RW, LCD_EN, LCD_RS,
	//GPIO
	inout [35:0] GPIO_0, GPIO_1
	);
	assign GPIO_1 = 36'hzzzzzzzzz;
	assign GPIO_0 = 36'hzzzzzzzzz;
	assign LCD_ON = 1'b1;
	assign LCD_BLON = 1'b1;
	
	//Interface LCD 16x2 
	logic [7:0] w_d0x0, w_d0x1, w_d0x2, w_d0x3, w_d0x4, w_d0x5,	
					w_d1x0, w_d1x1, w_d1x2, w_d1x3, w_d1x4, w_d1x5;
	LCD_6x2 MyLCD (
		.iCLK ( CLOCK_50 ),
		.iRST_N ( KEY[0] ),
		.d0x0(w_d0x0),.d0x1(w_d0x1),.d0x2(w_d0x2),.d0x3(w_d0x3),.d0x4(w_d0x4),.d0x5(w_d0x5),
		.d1x0(w_d1x0),.d1x1(w_d1x1),.d1x2(w_d1x2),.d1x3(w_d1x3),.d1x4(w_d1x4),.d1x5(w_d1x5),
		.LCD_DATA( LCD_DATA ),
		.LCD_RW ( LCD_RW ),
		.LCD_EN ( LCD_EN ),
		.LCD_RS ( LCD_RS )
	);

	// DEBUG UART
	logic [31:0] probe_bus [7:0];
	assign probe_bus[0] = 32'h00_00_00_00;
	assign probe_bus[1] = 32'h00_00_00_01;
	assign probe_bus[2] = 32'h00_00_00_02;
	assign probe_bus[3] = 32'h00_00_00_03;
	assign probe_bus[4] = 32'h00_00_00_04;
	assign probe_bus[5] = 32'h00_00_00_05;
	assign probe_bus[6] = 32'h00_00_00_06;
	assign probe_bus[7] = 32'h00_00_00_07;
	debug_dump_ctrl #(
		.CLK_FREQ_HZ       (50_000_000),
		.BAUD_RATE         (115_200),
		.NUM_REGS          (32),
		.NUM_PROBES        (8),
		.IMM_SRC_WIDTH     (2),
		.ULA_CONTROL_WIDTH (3),
		.RESULT_SRC_WIDTH  (2)) 
		myDebug (
		.clk_50(CLOCK_50),       	// clock da placa, alimenta soh a logica de debug
		.rst_n(KEY[0]),				// reset 
		.clk_cpu(clk_1hz),			// clock do CPU dos alunos (<=10Hz), soh observado, nunca gerado aqui
		// Sinais a serem monitorados
		.pc					(w_PC),
		.instruction		(w_Inst),
		.alu_result			(w_ULAResult),
		.debug_reg_data	(registers),// banco de 32 registradores, no formato "[31:0]registers[31:0]"
		.debug_probe		(probe_bus),// 8 canais genericos para observar sinais internos, no formato "[31:0] debug_probe [7:0]" 
		.reg_write			(w_RegWrite),
		.imm_src				(w_ImmSrc),
		.ula_src				(w_ULASrc),
		.ula_control		(w_ULAControl),
		.mem_write			(w_MemWrite),
		.result_src			(w_ResultSrc),
		.branch				(w_Branch),
		// saida UART fisica
		.uart_txd(GPIO_0[0])
	);

// Barramentos
	logic [31:0] registers [31:0];
   wire [31:0] w_ULAResult, w_PC, w_Inst;
	wire [2:0] w_ULAControl;
	wire [1:0] w_ImmSrc;
	wire w_ULASrc, w_RegWrite, w_ResultSrc, w_MemWrite, w_Branch;

// Código do processador RiscV:
//..

	
endmodule
