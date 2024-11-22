#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "wmap.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h"

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

extern struct mmap_regions *mmap;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);

  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

// Helper Functions:

// Finds out whether the fault address is in the linked list, changes linked list node to the one that contains it
int linked_list_check(uint addr, struct mmap_regions **node) {
  while(*node) {
    uint start = (*node)->block_start;
    uint end = (*node)->block_start + (*node)->block_size;

    // Find out if page fault is from an existing mapped region
    if (addr >= start && addr < end) {
      return SUCCESS;
    }
    *node = (*node)->next;
  }
  return FAILED;
}

// Allocates a new page and maps the faulting page to it. Returns pointer to mapped page
char * map_page(uint addr, struct mmap_regions *node) {
  // Get a free page
  char *mem = kalloc();
  if (mem == 0) {
    return 0;
  }

  memset(mem, 0, PGSIZE);

  // Map virtual address to page
  if (mappages(myproc()->pgdir, (void*)PGROUNDDOWN(addr), PGSIZE, V2P(mem), PTE_W | PTE_U) < 0) {
    kfree(mem);
    return 0;
  }

  node->allocated_count++;
  return mem;
}

// Checks if valid file exists, then loads correct offset from file into just allocated page
int load_page_from_file(uint addr, struct mmap_regions *node) {
  // Read memory from file
  if(!(node->f)) return SUCCESS;

  uint offset = PGROUNDDOWN(addr) - node->block_start;
  ilock(node->f->ip);
  if(readi(node->f->ip, (char*)PGROUNDDOWN(addr), offset, PGSIZE) < 0) {
    iunlock(node->f->ip);
    return FAILED;
  }
  iunlock(node->f->ip);
  return SUCCESS;

}

//PAGEBREAK: 41
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(myproc()->killed)
      exit();
    myproc()->tf = tf;
    syscall();
    if(myproc()->killed)
      exit();
    return;
  }

  switch(tf->trapno){
  case T_IRQ0 + IRQ_TIMER:
    if(cpuid() == 0){
      acquire(&tickslock);
      ticks++;
      wakeup(&ticks);
      release(&tickslock);
    }
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE:
    ideintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE+1:
    // Bochs generates spurious IDE1 interrupts.
    break;
  case T_IRQ0 + IRQ_KBD:
    kbdintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_COM1:
    uartintr();
    lapiceoi();
    break;
  case T_IRQ0 + 7:
  case T_IRQ0 + IRQ_SPURIOUS:
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpuid(), tf->cs, tf->eip);
    lapiceoi();
    break;
  case T_PGFLT:
    // cprintf("Page Fault: trapno=%d, eip=0x%x, cr2=0x%x, err=%d\n", tf->trapno, tf->eip, rcr2(), tf->err);

    uint fault_addr = rcr2();
    struct mmap_regions *current = mmap;

    if(!linked_list_check(fault_addr, &current)) {
      char *page;
      if(!(page = map_page(fault_addr, current))) {
        myproc()->killed = 1;
        break;
      }

      if(load_page_from_file(fault_addr, current)) {
        kfree(page);
        myproc()->killed = 1;
        break;
      }
    } else {
      // Address was NOT found in linked list
      myproc()->killed = 1;
      cprintf("Segmentation Fault\n");
    }
    break;

  //PAGEBREAK: 13
  default:
    if(myproc() == 0 || (tf->cs&3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpuid(), tf->eip, rcr2());
      panic("trap");
    }
    // In user space, assume process misbehaved.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            myproc()->pid, myproc()->name, tf->trapno,
            tf->err, cpuid(), tf->eip, rcr2());
    myproc()->killed = 1;
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running
  // until it gets to the regular system call return.)
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER) {
    exit();
  }


  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(myproc() && myproc()->state == RUNNING &&
     tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();

  // Check if the process has been killed since we yielded
  if(myproc() && myproc()->killed && (tf->cs&3) == DPL_USER)
    exit();
}