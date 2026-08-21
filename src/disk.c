#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "disk.h"

/* The disk image is the core's content, and on Android the frontend hands out
   SAF content:// URIs that no C library can open - so when a VFS interface is
   available every access to the image goes through it. Without one, and in the
   standalone build, this is plain stdio exactly as before. */
#ifdef __LIBRETRO__
#include <libretro.h>

static struct retro_vfs_interface *disk_vfs;

void disk_set_vfs_interface(struct retro_vfs_interface *iface) {
  disk_vfs = iface;
}

struct disk_file {
  struct retro_vfs_file_handle *vfs;
  FILE *stdio;
};

static struct disk_file *df_open(const char *path) {
  struct disk_file *df = calloc(1, sizeof(*df));
  if (!df) {
    return NULL;
  }
  if (disk_vfs) {
    df->vfs = disk_vfs->open(path,
                             RETRO_VFS_FILE_ACCESS_READ_WRITE |
                             RETRO_VFS_FILE_ACCESS_UPDATE_EXISTING,
                             RETRO_VFS_FILE_ACCESS_HINT_NONE);
    if (df->vfs) {
      return df;
    }
  }
  df->stdio = fopen(path, "rb+");
  if (!df->stdio) {
    free(df);
    return NULL;
  }
  return df;
}

static void df_seek(struct disk_file *df, int64_t offset) {
  if (!df) return;
  if (df->vfs) disk_vfs->seek(df->vfs, offset, RETRO_VFS_SEEK_POSITION_START);
  else fseek(df->stdio, (long)offset, SEEK_SET);
}

static void df_read(struct disk_file *df, void *buf, size_t len) {
  if (!df) return;
  if (df->vfs) disk_vfs->read(df->vfs, buf, len);
  else fread(buf, len, 1, df->stdio);
}

static void df_write(struct disk_file *df, const void *buf, size_t len) {
  if (!df) return;
  if (df->vfs) disk_vfs->write(df->vfs, buf, len);
  else fwrite(buf, len, 1, df->stdio);
}
#else
struct disk_file {
  FILE *stdio;
};

static struct disk_file *df_open(const char *path) {
  struct disk_file *df = calloc(1, sizeof(*df));
  if (!df) {
    return NULL;
  }
  df->stdio = fopen(path, "rb+");
  if (!df->stdio) {
    free(df);
    return NULL;
  }
  return df;
}

static void df_seek(struct disk_file *df, int64_t offset) {
  if (df) fseek(df->stdio, (long)offset, SEEK_SET);
}

static void df_read(struct disk_file *df, void *buf, size_t len) {
  if (df) fread(buf, len, 1, df->stdio);
}

static void df_write(struct disk_file *df, const void *buf, size_t len) {
  if (df) fwrite(buf, len, 1, df->stdio);
}
#endif

enum DiskState {
  diskCommand,
  diskRead,
  diskWrite,
  diskWriting,
};

struct Disk {
  struct RISC_SPI spi;

  enum DiskState state;
  struct disk_file *file;
  uint32_t offset;

  uint32_t rx_buf[128];
  int rx_idx;

  uint32_t tx_buf[128+2];
  int tx_cnt;
  int tx_idx;
};


static uint32_t disk_read(const struct RISC_SPI *spi);
static void disk_write(const struct RISC_SPI *spi, uint32_t value);
static void disk_run_command(struct Disk *disk);
static void seek_sector(struct disk_file *f, uint32_t secnum);
static void read_sector(struct disk_file *f, uint32_t buf[static 128]);
static void write_sector(struct disk_file *f, uint32_t buf[static 128]);


struct RISC_SPI *disk_new(const char *filename) {
  struct Disk *disk = calloc(1, sizeof(*disk));
  disk->spi = (struct RISC_SPI) {
    .read_data = disk_read,
    .write_data = disk_write
  };

  disk->state = diskCommand;

  if (filename) {
    disk->file = df_open(filename);
    if (disk->file == 0) {
      fprintf(stderr, "Can't open file \"%s\": %s\n", filename, strerror(errno));
#ifdef __LIBRETRO__
      /* A core must not take the frontend down with it - report the failure to
         the caller instead, which turns into a refused retro_load_game. */
      free(disk);
      return NULL;
#else
      exit(1);
#endif
    }

    // Check for filesystem-only image, starting directly at sector 1 (DiskAdr 29)
    read_sector(disk->file, &disk->tx_buf[0]);
    disk->offset = (disk->tx_buf[0] == 0x9B1EA38D) ? 0x80002 : 0;
  }

  return &disk->spi;
}

static void disk_write(const struct RISC_SPI *spi, uint32_t value) {
  struct Disk *disk = (struct Disk *)spi;
  disk->tx_idx++;
  switch (disk->state) {
    case diskCommand: {
      if ((uint8_t)value != 0xFF || disk->rx_idx != 0) {
        disk->rx_buf[disk->rx_idx] = value;
        disk->rx_idx++;
        if (disk->rx_idx == 6) {
          disk_run_command(disk);
          disk->rx_idx = 0;
        }
      }
      break;
    }
    case diskRead: {
      if (disk->tx_idx == disk->tx_cnt) {
        disk->state = diskCommand;
        disk->tx_cnt = 0;
        disk->tx_idx = 0;
      }
      break;
    }
    case diskWrite: {
      if (value == 254) {
        disk->state = diskWriting;
      }
      break;
    }
    case diskWriting: {
      if (disk->rx_idx < 128) {
        disk->rx_buf[disk->rx_idx] = value;
      }
      disk->rx_idx++;
      if (disk->rx_idx == 128) {
        write_sector(disk->file, &disk->rx_buf[0]);
      }
      if (disk->rx_idx == 130) {
        disk->tx_buf[0] = 5;
        disk->tx_cnt = 1;
        disk->tx_idx = -1;
        disk->rx_idx = 0;
        disk->state = diskCommand;
      }
      break;
    }
  }
}

static uint32_t disk_read(const struct RISC_SPI *spi) {
  struct Disk *disk = (struct Disk *)spi;
  uint32_t result;
  if (disk->tx_idx >= 0 && disk->tx_idx < disk->tx_cnt) {
    result = disk->tx_buf[disk->tx_idx];
  } else {
    result = 255;
  }
  return result;
}

static void disk_run_command(struct Disk *disk) {
  uint32_t cmd = disk->rx_buf[0];
  uint32_t arg = (disk->rx_buf[1] << 24)
    | (disk->rx_buf[2] << 16)
    | (disk->rx_buf[3] << 8)
    | disk->rx_buf[4];

  switch (cmd) {
    case 81: {
      disk->state = diskRead;
      disk->tx_buf[0] = 0;
      disk->tx_buf[1] = 254;
      seek_sector(disk->file, arg - disk->offset);
      read_sector(disk->file, &disk->tx_buf[2]);
      disk->tx_cnt = 2 + 128;
      break;
    }
    case 88: {
      disk->state = diskWrite;
      seek_sector(disk->file, arg - disk->offset);
      disk->tx_buf[0] = 0;
      disk->tx_cnt = 1;
      break;
    }
    default: {
      disk->tx_buf[0] = 0;
      disk->tx_cnt = 1;
      break;
    }
  }
  disk->tx_idx = -1;
}

static void seek_sector(struct disk_file *f, uint32_t secnum) {
  df_seek(f, (int64_t)secnum * 512);
}

static void read_sector(struct disk_file *f, uint32_t buf[static 128]) {
  uint8_t bytes[512] = { 0 };
  df_read(f, bytes, 512);
  for (int i = 0; i < 128; i++) {
    buf[i] = (uint32_t)bytes[i*4+0]
      | ((uint32_t)bytes[i*4+1] << 8)
      | ((uint32_t)bytes[i*4+2] << 16)
      | ((uint32_t)bytes[i*4+3] << 24);
  }
}

static void write_sector(struct disk_file *f, uint32_t buf[static 128]) {
  uint8_t bytes[512];
  if (!f) {
    return;
  }
  for (int i = 0; i < 128; i++) {
    bytes[i*4+0] = (uint8_t)(buf[i]      );
    bytes[i*4+1] = (uint8_t)(buf[i] >>  8);
    bytes[i*4+2] = (uint8_t)(buf[i] >> 16);
    bytes[i*4+3] = (uint8_t)(buf[i] >> 24);
  }
  df_write(f, bytes, 512);
}
