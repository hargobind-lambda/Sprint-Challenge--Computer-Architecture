#include <stdio.h>
#include "cpu.h"

/**
 * Main
 */
int main(int argc, char ** argv)
{
  struct cpu cpu;

  if (argc < 2 && argv[1] == NULL) {
    printf("ERROR: No file provided\n");
    return 1;
  }

  // printf("SP_START %u\n", SP_START);
  cpu_init(&cpu);
  cpu_load(&cpu, argv[1]);
  cpu_run(&cpu);

  return 0;
}