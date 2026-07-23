#!/usr/bin/env python3
"""
debug_monitor.py

Recebe pacotes de debug enviados pelo modulo debug_dump_ctrl (SystemVerilog)
via UART, decodifica o estado do processador RISC-V e exibe um dashboard
no terminal que atualiza a cada passo (curses).

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

Uso:
    Duplo-clique no executavel (ou rode sem argumentos) -> ele lista as
    portas seriais disponiveis e pede pra escolher uma.

    Tambem da pra especificar direto por linha de comando, se preferir:
        python3 debug_monitor.py --port COM5 --baud 115200
        python3 debug_monitor.py --port /dev/ttyUSB0 --baud 115200

Requer: pip install pyserial
"""

import argparse
import curses
import struct
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Erro: pyserial nao instalado. Rode: pip install pyserial")
    input("Pressione Enter para sair1...")
    sys.exit(1)

HEADER = 0xAA
NUM_REGS = 32
NUM_PROBES = 8
PKT_LEN = 1 + 4 + 4 + 4 + (NUM_REGS * 4) + (NUM_PROBES * 4) + 2 + 1  # = 176


def read_exact(ser, n):
    """Le exatamente n bytes da serial (bloqueante ate timeout)."""
    data = bytearray()
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            return None  # timeout
        data.extend(chunk)
    return bytes(data)


def sync_to_header(ser):
    """Descarta bytes ate encontrar o byte de header 0xAA."""
    while True:
        b = ser.read(1)
        if not b:
            return False  # timeout, tenta de novo no loop externo
        if b[0] == HEADER:
            return True


def parse_packet(ser):
    """Sincroniza no header e le/decodifica um pacote completo.
    Retorna dict com os campos, ou None se falhar (checksum errado etc)."""
    if not sync_to_header(ser):
        return None

    body = read_exact(ser, PKT_LEN - 1)  # resto do pacote, sem o header
    if body is None:
        return None

    pc, instr, alu = struct.unpack(">III", body[0:12])
    regs = struct.unpack(">32I", body[12:12 + NUM_REGS * 4])
    probes_start = 12 + NUM_REGS * 4
    probes = struct.unpack(">8I", body[probes_start:probes_start + NUM_PROBES * 4])
    control_start = probes_start + NUM_PROBES * 4
    (control_word,) = struct.unpack(">H", body[control_start:control_start + 2])
    checksum_rx = body[-1]

    checksum_calc = 0
    for byte in body[:-1]:
        checksum_calc ^= byte

    if checksum_calc != checksum_rx:
        return None  # pacote corrompido, descarta

    return {
        "pc": pc,
        "instr": instr,
        "alu": alu,
        "regs": regs,
        "probes": probes,
        "reg_write":    bool((control_word >> 15) & 0x1),
        "imm_src":            (control_word >> 13) & 0x3,
        "ula_src":      bool((control_word >> 12) & 0x1),
        "ula_control":        (control_word >> 9)  & 0x7,
        "mem_write":    bool((control_word >> 8)  & 0x1),
        "result_src":         (control_word >> 6)  & 0x3,
        "branch":       bool((control_word >> 5)  & 0x1),
    }


def draw_dashboard(stdscr, ser, trace_size=12):
    """
    Modo streaming automatico: a cada ciclo do CPU (<=10Hz), um pacote
    chega sozinho pela UART, sem precisar de nenhuma interacao do usuario.
    O dashboard mostra o snapshot atual + um trace das ultimas instrucoes,
    o que e bem util com programas de teste pequenos: da pra ver o
    programa inteiro rodar do começo ao fim.
    """
    curses.curs_set(0)
    stdscr.nodelay(False)
    trace = []       # historico circular de (step_num, pc, instr)
    step_count = 0
    reg_diff_prev = None  # regs do pacote anterior, para destacar o que mudou

    while True:
        pkt = parse_packet(ser)
        if pkt is None:
            continue  # timeout ou checksum invalido, tenta de novo (streaming continua sozinho)

        step_count += 1
        trace.append((step_count, pkt["pc"], pkt["instr"]))
        if len(trace) > trace_size:
            trace.pop(0)

        stdscr.erase()
        h, w = stdscr.getmaxyx()

        stdscr.addstr(0, 0, "=== RISC-V Single-Cycle :: Debug Monitor (streaming automatico) ===", curses.A_BOLD)
        stdscr.addstr(1, 0, f"Passo #{step_count}  |  Ctrl+C para sair", curses.A_DIM)

        stdscr.addstr(3, 0, f"PC          : 0x{pkt['pc']:08X}")
        stdscr.addstr(4, 0, f"Instruction : 0x{pkt['instr']:08X}")
        stdscr.addstr(5, 0, f"ULA result  : 0x{pkt['alu']:08X}  ({pkt['alu']})")
        stdscr.addstr(6, 0,
            f"Control : RegWrite={int(pkt['reg_write'])}  "
            f"ImmSrc={pkt['imm_src']:02b}  "
            f"ULASrc={int(pkt['ula_src'])}  "
            f"ULAControl={pkt['ula_control']:03b}  "
            f"MemWrite={int(pkt['mem_write'])}  "
            f"ResultSrc={pkt['result_src']:02b}  "
            f"Branch={int(pkt['branch'])}")

        stdscr.addstr(9, 0, "Registradores:", curses.A_BOLD)
        col_width = 16
        cols = 4 #max(1, w // col_width)
        for i in range(NUM_REGS):
            row = 10 + i // cols
            col = (i % cols) * col_width
            if row < h - 1:
                text = f"x{i:<2}=0x{pkt['regs'][i]:08X}"
                # destaca registradores que mudaram desde o passo anterior
                changed = (reg_diff_prev is not None and
                           reg_diff_prev[i] != pkt['regs'][i])
                attr = curses.A_REVERSE if changed else curses.A_NORMAL
                stdscr.addstr(row, col, text, attr)

        reg_diff_prev = pkt['regs']

        probes_row = 10 + (NUM_REGS + cols - 1) // cols + 1

        # painel das 8 sondas genericas (sinais internos do datapath)
        probe_col_width = 20
        probe_cols = 4 #max(1, w // probe_col_width)
        if probes_row < h - 1:
            stdscr.addstr(probes_row, 0, "Sondas (sinais internos):", curses.A_BOLD)
            for i, val in enumerate(pkt["probes"]):
                row = probes_row + 1 + i // probe_cols
                col = (i % probe_cols) * probe_col_width
                if row < h - 1:
                    stdscr.addstr(row, col, f"p{i}=0x{val:08X}")

        regs_end_row = probes_row + 1 + (NUM_PROBES + probe_cols - 1) // probe_cols + 1

        # painel de trace das ultimas instrucoes executadas
        if regs_end_row < h - 2:
            stdscr.addstr(regs_end_row, 0, "Trace (ultimas instrucoes):", curses.A_BOLD)
            for j, (n, tpc, tinstr) in enumerate(trace):
                row = regs_end_row + 1 + j
                if row < h - 1:
                    marker = "->" if j == len(trace) - 1 else "  "
                    stdscr.addstr(row, 0,
                        f"{marker} #{n:<5} PC=0x{tpc:08X}  instr=0x{tinstr:08X}")

        stdscr.refresh()

def choose_port_interactively():
    """Lista as portas seriais disponiveis e pede pro usuario escolher uma
    digitando o numero. Usado quando o script roda sem --port (ex: duplo-
    clique no executavel), pra nao precisar abrir terminal nem saber o
    nome da porta de antemao."""
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        print("Nenhuma porta serial encontrada.")
        print("Verifique se o cabo/adaptador USB-serial esta conectado")
        print("e se o driver dele esta instalado.")
        return None

    print("Portas seriais encontradas:\n")
    for i, p in enumerate(ports):
        desc = p.description if p.description else "(sem descricao)"
        print(f"  [{i}] {p.device}  -  {desc}")
    print()

    while True:
        choice = input(f"Escolha a porta (0-{len(ports)-1}): ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        print("Opcao invalida, tente de novo.")

def main():
    parser = argparse.ArgumentParser(description="Monitor de debug UART para RISC-V single-cycle na DE2")
    parser.add_argument("--port", default=None, help="Porta serial, ex: COM5 ou /dev/ttyUSB0. Se omitido, pede pra escolher.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (deve bater com o hardware)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout de leitura em segundos")
    args = parser.parse_args()

    print("=== RISC-V Single-Cycle :: Debug Monitor ===\n")

    port = args.port
    if port is None:
        port = choose_port_interactively()
        if port is None:
            input("\nPressione Enter para sair3...")
            sys.exit(1)

    try:
        ser = serial.Serial(port, args.baud, timeout=args.timeout)
    except serial.SerialException as e:
        print(f"\nErro ao abrir porta serial {port}: {e}")
        input("Pressione Enter para sair...")
        sys.exit(1)

    try:
        curses.wrapper(lambda stdscr: draw_dashboard(stdscr, ser))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        ser.close()
        print(f"\nErro inesperado: {e}")
        input("Pressione Enter para sair4...")
        sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
