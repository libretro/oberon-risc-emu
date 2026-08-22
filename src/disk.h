#ifndef DISK_H
#define DISK_H

#include "risc-io.h"

struct RISC_SPI *disk_new(const char *filename);

#ifdef __LIBRETRO__
struct retro_vfs_interface;
/* Hand over the frontend's VFS so the disk image is opened through it. Without
   one (or in the standalone build) the image is opened with stdio as before. */
void disk_set_vfs_interface(struct retro_vfs_interface *iface);
#endif

#endif  // DISK_H
