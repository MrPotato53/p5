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

int sys_wmap(void)
{
  uint addr;
  int length, flags, fd;

  if (argint(0, (int *)&addr) < 0 ||
      argint(1, &length) < 0 ||
      argint(2, &flags) < 0 ||
      argint(3, &fd) < 0)
    return FAILED;

  if (length <= 0)
    return FAILED;

  if (!(flags & MAP_FIXED))
    return FAILED;

  if (!(flags & MAP_SHARED))
    return FAILED;

  if (addr % PGSIZE != 0)
    return FAILED;

  if (addr < 0x60000000 || addr + length >= 0x80000000)
    return FAILED;

  if (flags & MAP_ANONYMOUS)
    fd = -1;
  else if (fd < 0 || fd >= NOFILE || myproc()->ofile[fd] == 0)
    return FAILED;

  uint new_start = addr;
  uint new_end = addr + length;

  struct mmap_regions *current = mmap;
  // Walk linked list to find if new mmap region intersects with existing regions
  while (current)
  {
    uint start = current->block_start;
    uint end = current->block_start + current->block_size;

    if (new_start < end && new_end > start)
      return FAILED;

    current = current->next;
  }

  if (myproc()->mmap_cnt >= MAX_WMMAP_INFO)
    return FAILED;

  // Add new mmap region to mmap linked list
  struct mmap_regions *new_region = (struct mmap_regions *)kalloc();
  if (!new_region)
    return FAILED;

  new_region->block_start = addr;
  new_region->block_size = length;
  new_region->next = mmap;
  new_region->pid = myproc()->pid;
  new_region->allocated_count = 0;
  new_region->f = fd > 0 ? filedup(myproc()->ofile[fd]) : 0;
  new_region->fd = fd;
  mmap = new_region;
  mmap_list_length++;
  myproc()->mmap_cnt++;

  return addr;
}

int sys_wunmap(void)
{
  uint addr;
  if (argint(0, (int *)&addr) < 0)
    return FAILED;
  if (addr % PGSIZE != 0)
    return FAILED;
  int map_found = 0;

  struct mmap_regions *current = mmap;
  struct mmap_regions *prev = 0;
  while (current)
  {
    if (addr == current->block_start)
    {
      map_found = 1;
      break;
    }

    prev = current;
    current = current->next;
  }

  if (!map_found)
    return FAILED;

  // Check that file descriptor is valid and that fd is referring to a valid open file
  struct file *f = 0;
  if(current->fd >= 0) {
    if(!(f = current->f)) {
      return FAILED;
    }
  }

  uint physical_address;
  uint offset;
  uint bytes_to_write;
  for (uint i = addr; i < addr + current->block_size; i += PGSIZE)
  {
    pte_t *pte = walkpgdir(myproc()->pgdir, (void *)i, 0);
    // Check to make sure page table entry exists and is valid
    if (pte && (*pte & PTE_P))
    {
      // Write page into file if page is allocated
      if(current->fd >= 0) {
        offset = i - addr;
        bytes_to_write = addr + current->block_size - i > PGSIZE ? PGSIZE : addr + current->block_size - i;
        ilock(f->ip);
        writei(f->ip, (char*)i, offset, PGSIZE);
        iunlock(f->ip);
      }

      physical_address = PTE_ADDR(*pte);
      kfree(P2V(physical_address));
      *pte = 0;
    }
  }

  prev->next = current->next;
  mmap_list_length--;
  myproc()->mmap_cnt--;
  kfree(P2V(current));
  return SUCCESS;
}

int sys_va2pa(void)
{
  uint addr;
  if (argint(0, (int *)&addr) < 0)
    return FAILED;

  pte_t *pte = walkpgdir(myproc()->pgdir, (void *)addr, 0);

  if (pte == 0 || !(*pte & PTE_P))
    return FAILED;

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
      (wminfo->addr)[list_index] = current->block_start;
      (wminfo->length)[list_index] = current->block_size;
      (wminfo->n_loaded_pages)[list_index] = current->allocated_count;
      list_index++;

      current = current->next;
    }
  }
  return SUCCESS;
}