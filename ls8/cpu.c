#include "cpu.h"
#include <stdio.h>
#include <stdlib.h>

void handle_CMP(struct cpu *cpu)
{
  unsigned char regA, regB;
  regA = cpu_register_read(cpu, cpu_ram_pc(cpu));
  regB = cpu_register_read(cpu, cpu_ram_read(cpu, cpu->pc + 1));
  if (regA == regB)
  {
    // regB is equal to regB
    cpu->FL = 1;//cpu->FL & CMP_E_MASK;
  }
  else if (regA > regB)
  {
    cpu->FL = 2;//cpu->FL & CMP_G_MASK;
  }
  else if (regA < regB)
  {
    cpu->FL = 4;//cpu->FL & CMP_L_MASK;
  }
#ifdef DEBUG
  printf("based on regA: %u and regB %u the cmp is set to %u\n", regA, regB, cpu->FL);
#endif
}

void handle_JEQ(struct cpu *cpu)
{
  unsigned char jmp_address = cpu_register_read(cpu, cpu_ram_pc(cpu));
  if (cpu->FL == CMP_E_MASK)
  {
#ifdef DEBUG
    printf("JEQ Was equal. Jumping to: %u\n", jmp_address);
#endif
    cpu->pc = jmp_address;
  }
  else
  {
#ifdef DEBUG
    printf("JEQ Was not equal. Continuing to: %u", cpu->pc + 1);
#endif
  cpu->pc += 1;
  }
}

void handle_JNE(struct cpu *cpu)
{
  unsigned char jmp_address = cpu_register_read(cpu, cpu_ram_pc(cpu));
  if ((cpu->FL & CMP_E_MASK) == 0)
  {
#ifdef DEBUG
    printf("JNE Was not equal. Jumping to: %u\n", jmp_address);
#endif
    cpu->pc = jmp_address;
  }
  else
  {
#ifdef DEBUG
    printf("JNE Was equal. Continuing to: %u", cpu->pc + 1);
#endif
  cpu->pc += 1;
  }
}

void handle_JMP(struct cpu *cpu)
{
  unsigned char jmp_address = cpu_register_read(cpu, cpu_ram_pc(cpu));
#ifdef DEBUG
  printf("JMP Jumping to: %u\n", jmp_address);
#endif
  cpu->pc = jmp_address;
}




void handle_LDI(struct cpu *cpu)
{
  /* LDI reg int */
#ifdef DEBUG
  printf("load %u into reg %u\n",
         cpu_ram_read(cpu, cpu->pc + 1),
         cpu_ram_read(cpu, cpu->pc));
#endif

  cpu_register_write(cpu,
                     cpu_ram_read(cpu, cpu->pc),
                     cpu_ram_read(cpu, cpu->pc + 1));
  // cpu->reg[cpu->ram[cpu->pc]] = cpu->ram[cpu->pc + 1];
  // cpu->pc += cpu_ram_read(cpu, cpu->pc-1) >> 6;
  // cpu->pc += 2;
}

void handle_MUL(struct cpu *cpu)
{

#ifdef DEBUG
  printf("multipling %02u and %02u \n", cpu_ram_read(cpu, cpu->pc), cpu_ram_read(cpu, cpu->pc + 1));
#endif

  alu(cpu, ALU_MUL, cpu_ram_read(cpu, cpu->pc), cpu_ram_read(cpu, cpu->pc + 1));
  // cpu->pc += cpu_ram_read(cpu, cpu->pc-1) >> 6;
}

void handle_ADD(struct cpu *cpu)
{

#ifdef DEBUG
  printf("adding %02u and %02u \n", cpu_ram_read(cpu, cpu->pc), cpu_ram_read(cpu, cpu->pc + 1));
#endif

  alu(cpu, ALU_ADD, cpu_ram_read(cpu, cpu->pc), cpu_ram_read(cpu, cpu->pc + 1));
}

void handle_HLT(int *running)
{
  *running = 0;
#ifdef DEBUG
  printf("END PROGRAM\n");
#endif
}

void handle_CALL(struct cpu *cpu)
{
  // push return address onto stack
  cpu_push_stack(cpu, cpu->pc + 1);

#ifdef DEBUG
  printf("Calling subroutine at address %u", cpu_register_read(cpu, cpu_ram_read(cpu, cpu->pc)));
#endif

  // move pc to subroutine address
  cpu->pc = cpu_register_read(cpu, cpu_ram_pc(cpu));
}

void handle_RET(struct cpu *cpu)
{

  // move pc to return address in stack
  cpu->pc = cpu_pop_stack(cpu);

#ifdef DEBUG
  printf("Returning to aaddress %3u", cpu->pc);
#endif
}
/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *filepath)
{
  int address = 0;
  FILE *fp = fopen(filepath, "r");

  if (!fp)
  {
    perror("File opening failed");
    // return 2;
  }

  char buf[1024];
  // read file loop
  // char *c;
  // unsigned char *ramp = cpu->ram;
  while (fgets(buf, 1024, fp) != NULL)
  {
    char *endptr;
    // *ramp = c;

    unsigned char val = strtoul(buf, &endptr, 2);
#ifdef DEBUG
    printf("line: %2u, loading ram: %4u\n", address, val);
#endif
    if (buf == endptr)
    {
      continue;
    }
    // putchar(c);

    cpu_ram_write(cpu, address++, val);
    // cpu->ram[address] = val;
    // address++;
    // ramp++;
  }

  // keeps track of where the program ends
  cpu->PROGRAM_SIZE = address;

#ifdef DEBUG
  if (ferror(fp))
    puts("I/O error when reading");
  else if (feof(fp))
    puts("End of file reached successfully");
#endif

  fclose(fp);
}

/**
 * ALU
 */
void alu(struct cpu *cpu, enum alu_op op, unsigned char regA, unsigned char regB)
{
  unsigned char result = 0;
  switch (op)
  {
  case ALU_MUL:
    result = cpu_register_read(cpu, regA) * cpu_register_read(cpu, regB);
    cpu_register_write(cpu, regA, result);
    break;
    // TODO: implement more ALU ops
  case ALU_ADD:
    result = cpu_register_read(cpu, regA) + cpu_register_read(cpu, regB);
    cpu_register_write(cpu, regA, result);
    break;
  
  case ALU_CMP:
    break;
  }
}

#ifdef DEBUG
void cpu_print_state(struct cpu *cpu)
{
  printf("\ncpu->pc: %03u op: %3u, registers: [ ", cpu->pc+1, cpu->ram[cpu->pc]);
  for (int i = 0; i < 8; i++)
  {
    printf("%3u ", cpu->reg[i]);
  }
  printf("]\n");
}
#endif

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction
  unsigned char cur_ins;
  // unsigned char reg_0, reg_1, reg_2;
  // unsigned char val;
  unsigned int num_err = 0;

  while (running)
  {

#ifdef DEBUG
    cpu_print_state(cpu);
#endif

    if (num_err > 5)
    {
      printf("too many bad instructions: %u. Ending emulation\n", num_err);
      break;
    }

    // TODO
    // 1. Get the value of the current instruction (in address PC).
    cur_ins = cpu_ram_read(cpu, cpu->pc++);
    // 2. Figure out how many operands this next instruction requires
    unsigned int num_ops = cur_ins >> 6;
    unsigned char uses_ALU = cur_ins & ALU_MASK;
    unsigned char affects_PC = cur_ins & PC_MASK;
    // unsigned char op_id = cur_ins & OP_ID_MASK;
    // 3. Get the appropriate value(s) of the operands following this instruction
    // 4. switch() over it to decide on a course of action.
    switch (cur_ins)
    {
    // 5. Do whatever the instruction should do according to the spec.
    case HLT:
      handle_HLT(&running);
      break;

    case PRN:
      handle_PRN(cpu);
      break;

    case LDI:
      handle_LDI(cpu);
      break;

    case CALL:
      handle_CALL(cpu);
      break;

    case RET:
      handle_RET(cpu);
      break;

    case MUL:
      handle_MUL(cpu);
      break;

    case ADD:
      handle_ADD(cpu);
      break;

    case POP:
      // val = cpu_pop_stack(cpu);
      handle_POP(cpu);
      // cpu_register_write(cpu, cpu_ram_read(cpu, cpu->pc), val);
      break;

    case PUSH:
      // val = cpu_register_read(cpu, cpu_ram_read(cpu, cpu->pc));
      handle_PUSH(cpu);
      // printf("pushing %u onto the stack from register %u\n", val, cpu_ram_read(cpu, cpu->pc));
      break;

    case CMP:
      handle_CMP(cpu);
      break;

    case JEQ:
      handle_JEQ(cpu);
      break;
    case JNE:
      handle_JNE(cpu);
      break;
    case JMP:
      handle_JMP(cpu);
      break;

    default:
      printf("Invalid instruction: %u\n", cur_ins);
      num_err++;
      cpu->pc++;
      break;
    }
    // 6. Move the PC to the next instruction.
    // increment program counter based on instruction
    // quick fix to get call working
    if (!affects_PC)
    {
      cpu->pc += cpu_ram_read(cpu, cpu->pc - 1) >> 6;
    }
    // cpu->pc++;
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  // TODO: Initialize the PC and other special registers
  // cpu->ram;
  cpu->pc = 0;
  // cpu->reg[7] = MEM_SIZE - 12; // this is where the stack pointer is supposed to live
  cpu->reg[7] = sizeof(cpu->ram) / sizeof(cpu->ram[0]) - 12; // SP_START;
  cpu->FL = 0;
  // cpu->sp = &cpu->reg[7]; // reference to stack pointer register address
}

// helper functions

// Returns whatever is in ram at wherever the program counter is.
unsigned char cpu_ram_pc(struct cpu *cpu)
{
  return cpu->ram[cpu->pc];
}

unsigned char cpu_ram_read(struct cpu *cpu, unsigned int ram_index)
{
  return cpu->ram[ram_index];
}

void cpu_ram_write(struct cpu *cpu, unsigned int ram_index, unsigned char value)
{
  cpu->ram[ram_index] = value;
}

unsigned char cpu_register_read(struct cpu *cpu, unsigned int reg_i)
{
  return cpu->reg[reg_i];
}

void cpu_register_write(struct cpu *cpu, unsigned int reg_i, unsigned char value)
{
  cpu->reg[reg_i] = value;
}

// operation handlers
void handle_PRN(struct cpu *cpu)
{
  // unsigned int value = cpu->ram[cpu->pc++];
#ifdef DEBUG
  printf("print reg: %u ", cpu_ram_read(cpu, cpu->pc));
  printf("\"");
  printf("%u", cpu_register_read(cpu,
                                 cpu_ram_read(cpu, cpu->pc)));
#endif

#ifndef DEBUG
  printf("%u\n", cpu_register_read(cpu,
                                   cpu_ram_read(cpu, cpu->pc)));
#endif
#ifdef DEBUG
  printf("\"\n");
#endif
}
void handle_PUSH(struct cpu *cpu)
{

  unsigned char val;
  val = cpu_register_read(cpu, cpu_ram_read(cpu, cpu->pc));

#ifdef DEBUG
  printf("pushing %u onto the stack from register %u\n", val, cpu_ram_read(cpu, cpu->pc));
#endif
  cpu_push_stack(cpu, val);
}

void cpu_push_stack(struct cpu *cpu, unsigned char value)
{
  if (cpu->reg[7] >= cpu->PROGRAM_SIZE)
  {
    cpu->reg[7]--; // move to next empty stack location
    cpu_ram_write(cpu, cpu->reg[7], value);
  }
}

void handle_POP(struct cpu *cpu)
{
  unsigned char popped_value = cpu_pop_stack(cpu);
  cpu_register_write(cpu, cpu_ram_read(cpu, cpu->pc), popped_value);
#ifdef DEBUG
  printf("popping %u off the stack into register %u\n",
         popped_value, cpu_ram_read(cpu, cpu->pc));
#endif
}

unsigned char cpu_pop_stack(struct cpu *cpu)
{
  unsigned char popped_value = cpu_ram_read(cpu, cpu->reg[7]);
  // makes sure we can't pop past the sp start pos
  if (cpu->reg[7] <= SP_START)
  {
    cpu->reg[7]++; // move sp to previous value on stack
  }
  return popped_value;
}