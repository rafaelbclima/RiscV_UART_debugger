
# RiscV_UART_debugger
Framework para debug de um processador RiscV, implementado em System Verilog, via UART

----------------------------------------------------------------------------------
-- Para modificar o código do PC                                                --
----------------------------------------------------------------------------------
- Instalar o python
	- python.org/downloads
	- Na primeira tela do instalador, marque a caixinha "Add python.exe to PATH" 
	- Abra o prompt de comando do windows
	- Teste a versão do python com o comando: "python --version"
- Instalar "pip install pyserial"
- Instalar "pip install windows-curses"
- Editar o debug_monitor.py no bloco de notas
- Teste o código rodando o prompt de comando do Windows: "python debug_monitor.py"

----------------------------------------------------------------------------------
-- Para gerar o .exe e rodar no PC                                              --
----------------------------------------------------------------------------------
- Instale "pip install pyserial windows-curses pyinstaller"
- Rode o comando para gerar o .exe na pasta \dist: "pyinstaller --onefile --console --name DebugMonitor debug_monitor.py"
- Teste o executável clicando no arquivo "DebugMonitor.exe"
- Se certifique que o windows defender não bloqueou o .exe
<img width="543" height="244" alt="1" src="https://github.com/user-attachments/assets/5faabd33-b5bc-4144-96ea-0ea43a5e907a" />
<img width="894" height="643" alt="2" src="https://github.com/user-attachments/assets/7a101082-cfdb-46d2-9b66-2e65747db88a" />

----------------------------------------------------------------------------------
-- Para inclui o pacote de debug no HDL do seu processador RiscV                --
----------------------------------------------------------------------------------
- Inclua o arquivo utilidades.sv no seu projeto

----------------------------------------------------------------------------------
-- Para utilizar a interface UART no HDL do seu processador RiscV               --
----------------------------------------------------------------------------------
- Instancie o módulo debug_dump_ctrl
- Exemplo no quartus II
	
```
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
	.pc		(w_PC),
	.instruction	(w_Inst),
	.alu_result	(w_ULAResult),
	.debug_reg_data	(registers),// banco de 32 registradores, no formato "[31:0]registers[31:0]"
	.debug_probe	(probe_bus),// 8 canais genericos para observar sinais internos, no formato "[31:0] debug_probe [7:0]" 
	.reg_write	(w_RegWrite),
	.imm_src	(w_ImmSrc),
	.ula_src	(w_ULASrc),
	.ula_control	(w_ULAControl),
	.mem_write	(w_MemWrite),
	.result_src	(w_ResultSrc),
	.branch		(w_Branch),
	// saida UART fisica
	.uart_txd	(GPIO_0[0])
);

// LOGICS
	logic [31:0] registers [31:0];
// FIOS
wire [31:0] w_PCp4, w_ImmPC, w_PCn, w_rd1SrcA, w_rd2, w_SrcB, w_ULAResult, w_PC, w_Inst, w_Imm, w_RData, w_MImm;
wire [2:0] w_ULAControl;
wire [7:0] w_Wd3, w_muxImm1, w_muxImm2, w_muxImm3, w_DataOut, w_DataIn, w_RegData;
wire [1:0] w_ImmSrc;
wire w_ULASrc, w_RegWrite, w_ResultSrc, w_MemWrite, w_Branch, w_Zero, w_PCSrc;
```
----------------------------------------------------------------------------------
-- Formato do pacote utilizado pela interface UART                                               --
----------------------------------------------------------------------------------
Formato do pacote (176 bytes, big-endian):
    [0]        HEADER      = 0xAA
    [1:5]      PC          (32 bits)
    [5:9]      INSTRUCTION (32 bits)
    [9:13]     ALU_RESULT  (32 bits)
    [13:141]   REGFILE     (32 registradores x31..x0, 32 bits cada)
    [141:173]  PROBES      (8 sondas genericas, 32 bits cada)
    [173:175]  CONTROL     (16 bits, ver empacotamento abaixo)
    [175]      CHECKSUM    (XOR dos bytes 1..175)

Empacotamento do CONTROL (16 bits, MSB->LSB):
    bit 15    : RegWrite
    bits 14:13: ImmSrc[1:0]
    bit 12    : ULASrc
    bits 11:9 : ULAControl[2:0]
    bit 8     : MemWrite
    bits 7:6  : ResultSrc[1:0]
    bit 5     : Branch
    bits 4:0  : reservado

----------------------------------------------------------------------------------
-- Para utilizar a interface do LCD da placa Altera DE2                                               --
----------------------------------------------------------------------------------
- instancie o módulo LCD_6x2.sv
- Exemplo no quartus II

```
//Interface LCD 16x2 
assign LCD_ON = 1'b1;
assign LCD_BLON = 1'b1;
logic [7:0] 	w_d0x0, w_d0x1, w_d0x2, w_d0x3, w_d0x4, w_d0x5,	
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
```


