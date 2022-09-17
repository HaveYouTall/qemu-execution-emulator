#include "qemu/osdep.h"

#include "disas/disas.h"
#include "exec/exec-all.h"
#include "tcg/tcg-op.h"
#include "cpu.h"
#include "exec/translator.h"

#include <sys/mman.h>

// 测试的时候发现，指令插桩成功了，但是相关的内存操作的值并没有成功写到这些地方。
// 花了半天用 gdb 调试，终于断到了 翻译后的代码，然后发现写入操作执行时对的，但是确实没有写到正确的地方。
// 然后搜索这个值写到哪里了，发现跟实际值差了0x10000的偏移。
// 因为 插桩的指令 类似于 mov dword ptr gs:[rbx], r13 
// 所以我猜测 可能是gs 带来的影响（虽然我看了gs寄存器是0呀....）
// 最终我加了这个OFFSET，成功解决了 问题。
#define GS_OFFSET 0X10000


// R/W Segment, Fixed address. Test Ok.
uint32_t *mem_w = 0x3f850000; //0x2025a0;
uint32_t *mem_r = 0x3f860000; // + 0x10000;
uint32_t *ret_value = 0x3f870000; //+ 0x20000;
// uint32_t *mem_w = NULL; // = 0x3f810000; //0x2025a0;
// uint32_t *mem_r = NULL; // = 0x3f810000 + 0x10000;
// uint32_t *ret_value = NULL; // = 0x3f810000 + 0x20000;
// uint32_t test[0x1000] = {0};
size_t icount_mem_w = 0;
size_t icount_mem_r = 0;
size_t icount_ret_value = 0;

size_t next_start_mem_w = 0;
size_t next_start_mem_r = 0;
size_t next_start_ret_value = 0;


// void instrumenting_init() {
    // scanf("%d", &icount_mem_r);
    // 0x140014000

    // mem_w = (uint32_t*)(mmap(
    //                 /*0xa000a000,*/                   //分配的首地址
    //                 0x140014000,
    //                 0x10000,          //分配内存大小(必须是页的整数倍, 32位1页=4k)
    //                 PROT_READ | PROT_WRITE, //映射区域保护权限：读|写
    //                 MAP_ANONYMOUS | MAP_SHARED,  //匿名映射(不涉及文件io), 后面两个参数忽略
    //                 0,                      //要映射到内存中的文件描述符
    //                 0                       //文件映射的偏移量，通常设置为0，必须是页的整数倍
    // ));

    // mem_r = (uint32_t*)(mmap(
    //                 0x140024000,                   //分配的首地址
    //                 0x10000,          //分配内存大小(必须是页的整数倍, 32位1页=4k)
    //                 PROT_READ | PROT_WRITE, //映射区域保护权限：读|写
    //                 MAP_ANONYMOUS | MAP_SHARED,  //匿名映射(不涉及文件io), 后面两个参数忽略
    //                 0,                      //要映射到内存中的文件描述符
    //                 0                       //文件映射的偏移量，通常设置为0，必须是页的整数倍
    // ));


    // printf("w: 0x%lx, r: 0x%lx\n", mem_w, mem_r);
// }


// void mem_rw_instrumenting(TCGOpcode opc) {


// // Push eax

// // if is mem read
// if(opc == INDEX_op_qemu_ld_i32) {

//     // val = insn_get(env, s, MO_32);
//     printf("0x%x\n", (uint32_t)mem_r);
//     // reg = (b & 7) | REX_B(s);
//     // tcg_gen_movi_tl(s->T0, ((uint32_t)mem_r) & MASK32);
//     // gen_op_mov_reg_v(s, ot, reg, s->T0);

//     // Mov eax, mem_r

//     // add eax, icount

//     // Mov DWORD PTR [eax], mem_r_value
// } else if (opc == INDEX_op_qemu_st_i32) { // if is mem write

//     // Mov eax, mem_r

//     // add eax, icount

//     // Mov DWORD PTR [eax], mem_r_value
// } else {
//     printf("[Warning] Unsupported opc: %d\n", (uint32_t)opc);
// }

// // inc icount
// // Pop eax
// }

// static void find_func_name(target_ulong orig_addr) {
//     lookup_symbol(orig_addr);
// }

void arg_instrumenting(/*DisasContextBase *sbase*/target_ulong orig_addr, char* funcname, int arg_num) {
    if(!strcmp(funcname, lookup_symbol(orig_addr))) {
        
    }
}


void show_info() {
    FILE *logfile = fopen("/seagate/hyt/binary-sim/test/env_regs_test/logfile/mem_write_log.txt", "a");
    uint8_t *tmp = ((uint8_t*)(mem_w)) + GS_OFFSET;
    if (logfile) {
        fprintf(logfile, "Write Value:\n");
        for(size_t i = next_start_mem_w; i<icount_mem_w; i++) {
            fprintf(logfile, "address: 0x%lx, Value: 0x%x\n", ((uint32_t*)tmp) + i ,*(((uint32_t*)tmp) + i));
        }
        fprintf(logfile, "\n");
        fclose(logfile);
        next_start_mem_w = icount_mem_w;
    }

    logfile = fopen("/seagate/hyt/binary-sim/test/env_regs_test/logfile/mem_read_log.txt", "a");
    tmp = ((uint8_t*)(mem_r)) + GS_OFFSET;
    if (logfile) {
        fprintf(logfile, "Read Value:\n");
        for(size_t i = next_start_mem_r; i<icount_mem_r; i++) {
            // fprintf(logfile, "address: 0x%lx, Value: 0x%x\n", mem_r + i, *(mem_r + i));
            fprintf(logfile, "address: 0x%lx, Value: 0x%x\n", ((uint32_t*)tmp) + i ,*(((uint32_t*)tmp) + i));
        }
        fprintf(logfile, "\n");
        fclose(logfile);
        next_start_mem_r = icount_mem_r;
    }
}

