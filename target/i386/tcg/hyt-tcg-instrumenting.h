#ifndef _HYT_TCG_INSTRUMENTING_
#define _HYT_TCG_INSTRUMENTING_

#include "qemu/osdep.h"

// #include "disas/disas.h"
#include "tcg/tcg-op.h"
// #include "cpu.h"

// void mem_rw_instrumenting(TCGOpcode opc);

extern uint32_t *mem_w;
extern uint32_t *mem_r;
extern uint32_t *ret_value;
// extern uint32_t test[0x1000];
extern size_t icount_mem_w;
extern size_t icount_mem_r;
extern size_t icount_ret_value;

extern size_t next_start_mem_w;
extern size_t next_start_mem_r;
extern size_t next_start_ret_value;

#define MASK32  0xffffffff
#define INSTRUMENT 0x1999
// #define MOV_Ev_Iv 0X2022

void show_info();
// void instrumenting_init();

#endif