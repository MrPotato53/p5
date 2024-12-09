#include "types.h"
#include "x86.h"
#include "defs.h"
#include "date.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "file.h"
#include "wmap.h"

struct mmap_regions *mmap = 0;
int mmap_list_length = 0;

// wunmap helpers (need to be above exit)
int block_found_and_remove(uint addr, struct mmap_regions **current, struct mmap_regions **prev) {
  while (*current)
  {
    if (addr == (*current)->block_start)
    {
      if(*prev && *current) {
        (*prev)->next = (*current)->next;
      } else if(*current) {
        mmap = (*current)->next;
      }

      mmap_list_length--;
      myproc()->mmap_cnt--;
      return SUCCESS;
    }

    *prev = *current;
    *current = (*current)->next;
  }
  return FAILED;
}

int write_page_to_file(struct file *f, uint block_start, uint page_addr, struct mmap_regions *current) {
  
  uint offset;
  uint bytes_to_write;
  
  if(f) {
    offset = page_addr - block_start;
    bytes_to_write = (block_start + current->block_size - page_addr < PGSIZE) ? block_start + current->block_size - page_addr : PGSIZE;
    ilock(f->ip);
    begin_op();
    if(writei(f->ip, (char*)page_addr, offset, bytes_to_write) < bytes_to_write) {
      end_op();
      iunlock(f->ip);
      return FAILED;
    }
    end_op();
    iunlock(f->ip);
  }
  return SUCCESS;
}

int unmap_pages(uint addr, struct mmap_regions *current) {
  uint physical_address;

  for (uint i = addr; i < addr + current->block_size; i += PGSIZE)
  {
    pte_t *pte = walkpgdir(myproc()->pgdir, (void *)i, 0);
    // Check to make sure page table entry exists and is valid
    if (pte && (*pte & PTE_P))
    {
      // Write page into file if page is allocated
      if (write_page_to_file(current->f, addr, i, current) == FAILED) return FAILED;

      physical_address = PTE_ADDR(*pte);
      kfree(P2V(physical_address));
      *pte = 0;
    }
  }
  return SUCCESS;
}

int unmap_helper(uint addr) {

  struct mmap_regions *current = mmap;
  struct mmap_regions *prev = 0;

  if (block_found_and_remove(addr, &current, &prev) == FAILED) return FAILED;
  if (unmap_pages(addr, current) == FAILED) return FAILED;

  kfree((char *)current);
  return SUCCESS;
}

int sys_fork(void)
{
  return fork();
}

int sys_exit(void)
{
  exit();
  return 0; // not reached
}

int sys_wait(void)
{
  return wait();
}

int sys_kill(void)
{
  int pid;

  if (argint(0, &pid) < 0)
    return -1;
  return kill(pid);
}

int sys_getpid(void)
{
  return myproc()->pid;
}

int sys_sbrk(void)
{
  int addr;
  int n;

  if (argint(0, &n) < 0)
    return -1;
  addr = myproc()->sz;
  if (growproc(n) < 0)
    return -1;
  return addr;
}

int sys_sleep(void)
{
  int n;
  uint ticks0;

  if (argint(0, &n) < 0)
    return -1;
  acquire(&tickslock);
  ticks0 = ticks;
  while (ticks - ticks0 < n)
  {
    if (myproc()->killed)
    {
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

// return how many clock tick interrupts have occurred
// since start.
int sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

int failed_conditions(uint addr, int length, int flags, int *fd) {
  // Check length conditions
  if(length <= 0) return FAILED;
  // Check flags conditions
  if(!(flags & MAP_FIXED) || !(flags & MAP_SHARED)) return FAILED;
  // Check address conditions
  if(addr % PGSIZE) return FAILED;
  if (addr < 0x60000000 || addr + length >= 0x80000000) return FAILED;
  // Check file conditions
  if (flags & MAP_ANONYMOUS)
    *fd = -1;
  else if (*fd < 0 || *fd >= NOFILE || myproc()->ofile[*fd] == 0)
    return FAILED;
  return SUCCESS;
}

int has_overlap(uint addr, int length) {
  uint new_start = addr;
  uint new_end = addr + length;

  struct mmap_regions *current = mmap;
  // Walk linked list to find if new mmap region intersects with existing regions
  while (current)
  {
    uint start = current->block_start;
    uint end = current->block_start + current->block_size;

    if (new_start < end && new_end >= start) return FAILED;
    if(new_start <= start && new_end >= end) return FAILED;

    current = current->next;
  }
  return SUCCESS;
}

// Add new region to Linked List
int map(uint addr, int length, int fd) {
  // Check that proc has room for more mmaps
  if (myproc()->mmap_cnt >= MAX_WMMAP_INFO) return FAILED;

  // Insert node
  struct mmap_regions *new_region = (struct mmap_regions *)kalloc();
  if (!new_region)
    return FAILED;

  new_region->block_start = addr;
  new_region->block_size = length;
  new_region->pid = myproc()->pid;
  new_region->allocated_count = 0;
  new_region->f = fd >= 0 ? filedup(myproc()->ofile[fd]) : 0;
  new_region->next = mmap;
  mmap = new_region;
  mmap_list_length++;
  myproc()->mmap_cnt++;

  return SUCCESS;
}

int sys_wmap(void)
{
  uint addr;
  int length, flags, fd;

  if (argint(0, (int *)&addr) < 0 ||
      argint(1, &length) < 0 ||
      argint(2, &flags) < 0 ||
      argint(3, &fd) < 0)
    return FAILED;

  if(failed_conditions(addr, length, flags, &fd)) return FAILED;
  if(has_overlap(addr, length)) return FAILED;
  if(map(addr, length, fd)) return FAILED;

  return addr;
}

// wunmap helpers at top of file
int sys_wunmap(void)
{
  uint addr;
  if (argint(0, (int *)&addr) < 0)
    return FAILED;
  if (addr % PGSIZE != 0)
    return FAILED;
  
  return unmap_helper(addr);
}

int sys_va2pa(void)
{
  uint addr;
  if (argint(0, (int *)&addr) < 0)
    return -1;

  pte_t *pte = walkpgdir(myproc()->pgdir, (void *)addr, 0);

  if (pte == 0 || !(*pte & PTE_P))
    return -1;

  return PTE_ADDR(*pte) | (*pte & 0xFFF);
}

int sys_getwmapinfo(void)
{
  struct wmapinfo *wminfo;
  if (argptr(0, (char **)&wminfo, sizeof(struct wmapinfo)) < 0)
    return FAILED;

  wminfo->total_mmaps = myproc()->mmap_cnt;
  int list_index = 0;

  struct mmap_regions *current = mmap;
  while (current)
  {
    if (current->pid == myproc()->pid)
    {
      (wminfo->addr)[list_index] = (int)(current->block_start);
      (wminfo->length)[list_index] = (int)(current->block_size);
      (wminfo->n_loaded_pages)[list_index] = current->allocated_count;
      list_index++;
    }
    current = current->next;
  }
  return SUCCESS;
}