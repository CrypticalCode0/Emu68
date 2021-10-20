#include <boards.h>
#include <mmu.h>
#include <A64.h>
#include <devicetree.h>
#include <support.h>

__attribute__((aligned(4096)))
#include "./devicetree.h"

/*
    This is a Z3 ROM board with device tree resource. It provides userspace to read the keys and properties from
    the tree in order to e.g. find the available peripherals. The board is provided with its own m68k ROM with the
    resource inside. No ARM-side code is used in this board.
*/

static void map(struct ExpansionBoard *board)
{
    kprintf("[BOARD] Mapping ZIII devicetree board at address %08x\n", board->map_base);
    mmu_map(mmu_virt2phys((uintptr_t)board->rom_file), board->map_base, board->rom_size, MMU_ACCESS | MMU_ISHARE | MMU_ALLOW_EL0 | MMU_READ_ONLY | MMU_ATTR(0), 0);
    mmu_map(mmu_virt2phys((uintptr_t)dt_fdt_base()), board->map_base + board->rom_size, (dt_total_size() + 4095) & ~4095, MMU_ACCESS | MMU_ISHARE | MMU_ALLOW_EL0 | MMU_READ_ONLY | MMU_ATTR(0), 0);
}

static struct ExpansionBoard board = {
    devicetree_bin,
    4096,
    0,
    1,
    1,
    map
};

static void * __attribute__((used, section(".boards.z3"))) _board = &board;
