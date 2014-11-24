#ifndef _VPU_DMA_BUF_H_
#define _VPU_DMA_BUF_H_

#include "../include/vpu_mem.h"
#include "classmagic.h"

#define vpu_dmabuf_dev_FIELDS \
    int (*alloc)(struct vpu_dmabuf_dev *dev, size_t size, VPUMemLinear_t **data); \
    int (*free)(struct vpu_dmabuf_dev *dev, VPUMemLinear_t *idata); \
    int (*map)(struct vpu_dmabuf_dev *dev, int origin_fd, size_t size, VPUMemLinear_t **data); \
    int (*unmap)(struct vpu_dmabuf_dev *dev, VPUMemLinear_t *idata); \
    int (*share)(struct vpu_dmabuf_dev *dev, VPUMemLinear_t *idata, VPUMemLinear_t **out_data); \
    int (*reserve)(VPUMemLinear_t *idata, int origin_fd, void *priv); \
    int (*get_origin_fd)(VPUMemLinear_t *idata); \
    int (*get_fd)(VPUMemLinear_t *idata); \
    int (*get_ref)(VPUMemLinear_t *idata); \
    void* (*get_priv)(VPUMemLinear_t *idata);

typedef struct vpu_dmabuf_dev {
    vpu_dmabuf_dev_FIELDS
} vpu_dmabuf_dev;

inline int vpu_mem_judge_used_heaps_type();

int vpu_dmabuf_open(unsigned long align, struct vpu_dmabuf_dev **dev, char *title);
int vpu_dmabuf_close(struct vpu_dmabuf_dev *dev);

#endif
