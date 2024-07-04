#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/queue.h"
#include "target/riscv/cpu.h"
#include "tcg/tcg-op-common.h"
#include "tcg/tcg.h"
#include "tcg/tcg-temp-internal.h"
#include "tcg/tcg-op.h"
#include "exec/exec-all.h"
#include "exec/translator.h"
#include "checkpoint/checkpoint.h"
#include "exec/helper-proto-common.h"
#include <stdio.h>

#define HELPER_H "accel/tcg/checkpoint-helper.h"
#include "exec/helper-info.c.inc"
#undef HELPER_H

static void checkpoint_gen_empty_check_cb(void)
{
    TCGv_i32 cpu_index = tcg_temp_ebb_new_i32();
    TCGv_i64 udata = tcg_temp_ebb_new_i64();

    tcg_gen_movi_i64(udata, 0);
    tcg_gen_ld_i32(cpu_index, tcg_env,
                   - offsetof(ArchCPU, env) + offsetof(CPUState, cpu_index));

    gen_helper_checkpoint_sync_check(cpu_index,udata);

    tcg_temp_free_i64(udata);
    tcg_temp_free_i32(cpu_index);
}

void checkpoint_gen_empty_callback(void){
    tcg_gen_checkpoint_cb_start();
    checkpoint_gen_empty_check_cb();
    tcg_gen_checkpoint_cb_end();
}

// static int temp_index=0;
void helper_checkpoint_sync_check(uint32_t cpu_index, uint64_t udata) {
    MachineState *ms = MACHINE(qdev_get_machine());
    NEMUState *ns = NEMU_MACHINE(ms);
    ns->checkpoint_info.exec_insns[cpu_index] += udata;
//    printf("checkpoint sync check cpu index %d, udata %ld\n", cpu_index, udata);
    if (ns->checkpoint_info.checkpoint_mode == NoCheckpoint) {
        return;
    }
    
    try_take_cpt(ns, ns->checkpoint_info.exec_insns[cpu_index], cpu_index, false);
}


__attribute_maybe_unused__
static TCGOp *find_op(TCGOp *op, TCGOpcode opc)
{
    while (op) {
        if (op->opc == opc) {
            return op;
        }
        op = QTAILQ_NEXT(op, link);
    }
    return NULL;
}

__attribute_maybe_unused__
static TCGOp *copy_op_nocheck(TCGOp **begin_op, TCGOp *op)
{
    TCGOp *old_op = QTAILQ_NEXT(*begin_op, link);
    unsigned nargs = old_op->nargs;

    *begin_op = old_op;
    op = tcg_op_insert_after(tcg_ctx, op, old_op->opc, nargs);
    memcpy(op->args, old_op->args, sizeof(op->args[0]) * nargs);

    return op;
}

__attribute_maybe_unused__
static TCGOp *copy_op(TCGOp **begin_op, TCGOp *op, TCGOpcode opc)
{
    op = copy_op_nocheck(begin_op, op);
    tcg_debug_assert((*begin_op)->opc == opc);
    return op;
}

__attribute_maybe_unused__
static TCGOp *rm_ops_range(TCGOp *begin, TCGOp *end)
{
    TCGOp *ret = QTAILQ_NEXT(end, link);

    QTAILQ_REMOVE_SEVERAL(&tcg_ctx->ops, begin, end, link);
    return ret;
}

__attribute_maybe_unused__
static TCGOp *copy_call(TCGOp **begin_op, TCGOp *op, void* func)
{
    TCGOp *old_op;
    int func_idx;


    /* copy all ops until the call */
    do {
        op = copy_op_nocheck(begin_op, op);
    } while (op->opc != INDEX_op_call);

    /* fill in the op call */
    old_op = *begin_op;
    TCGOP_CALLI(op) = TCGOP_CALLI(old_op);
    TCGOP_CALLO(op) = TCGOP_CALLO(old_op);
    tcg_debug_assert(op->life == 0);

    func_idx = TCGOP_CALLO(op) + TCGOP_CALLI(op);
    op->args[func_idx] = (uintptr_t)func;

    return op;
}

__attribute_maybe_unused__
static void inject(TCGOp *begin_op, TCGOp *op, uint64_t num_insns)
{
    /* const_ptr */
    op = copy_op(&begin_op, op, INDEX_op_mov_i64);
    op->args[1] = tcgv_i64_arg(tcg_constant_i64(num_insns));

    /* copy the ld_i32, but note that we only have to copy it once */
    op = copy_op(&begin_op, op, INDEX_op_ld_i32);

    /* call */
    op = copy_call(&begin_op, op, &helper_checkpoint_sync_check);
}


// reference inject cb_type
__attribute_maybe_unused__
static void checkpoint_inject_num_insns(TCGOp* begin_op, uint64_t num_insns){
    TCGOp *end_op;
    TCGOp *op;

    end_op = find_op(begin_op, INDEX_op_checkpoint_cb_end);
    tcg_debug_assert(end_op);

    op = end_op;
    inject(begin_op, op, num_insns);
    rm_ops_range(begin_op, end_op);

}

void inject_checkpoint_cb(uint64_t num_insns){
    TCGOp *op;
    
    QTAILQ_FOREACH(op, &tcg_ctx->ops, link) {
        switch (op->opc) {
        case INDEX_op_checkpoint_cb_start:
            checkpoint_inject_num_insns(op, num_insns);
            break;
        default: break;
        }
    }
}
