/*
 * ispqueue.c
 *
 * Marvell DxO ISP - buffer queue module
 *	Based on omap3isp
 *
 * Copyright:  (C) Copyright 2011 Marvell International Ltd.
 *              Henry Zhao <xzhao10@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */


#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/poll.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "ispqueue.h"

static int isp_video_buffer_lock_vma(struct isp_video_buffer *buf, int lock)
{
	struct vm_area_struct *vma;
	unsigned long start;
	unsigned long end;
	int ret = 0;

	if (buf->vbuf.memory == V4L2_MEMORY_MMAP)
		return 0;

	if (!current || !current->mm)
		return lock ? -EINVAL : 0;

	start = buf->vbuf.m.userptr;
	end = buf->vbuf.m.userptr + buf->vbuf.length - 1;

	down_write(&current->mm->mmap_sem);
	spin_lock(&current->mm->page_table_lock);

	do {
		vma = find_vma(current->mm, start);
		if (vma == NULL) {
			ret = -EFAULT;
			goto out;
		}

		if (lock)
			vma->vm_flags |= VM_LOCKED;
		else
			vma->vm_flags &= ~VM_LOCKED;

		start = vma->vm_end + 1;
	} while (vma->vm_end < end);

	if (lock)
		buf->vm_flags |= VM_LOCKED;
	else
		buf->vm_flags &= ~VM_LOCKED;

out:
	spin_unlock(&current->mm->page_table_lock);
	up_write(&current->mm->mmap_sem);
	return ret;
}

static void isp_video_buffer_cleanup(struct isp_video_buffer *buf)
{
	unsigned int i;

	if (buf->queue->ops->buffer_cleanup)
		buf->queue->ops->buffer_cleanup(buf);

	if (buf->pages != NULL) {
		isp_video_buffer_lock_vma(buf, 0);

		for (i = 0; i < buf->npages; ++i) {
			if (buf->pages[i])
				page_cache_release(buf->pages[i]);
		}

		vfree(buf->pages);
		buf->pages = NULL;
	}

	buf->npages = 0;
}

static int isp_video_buffer_prepare_user(struct isp_video_buffer *buf)
{
	unsigned long data;
	unsigned int first;
	unsigned int last;
	int ret;

	data = buf->vbuf.m.userptr;
	first = (data & PAGE_MASK) >> PAGE_SHIFT;
	last = ((data + buf->vbuf.length - 1) & PAGE_MASK) >> PAGE_SHIFT;


	buf->offset = data & ~PAGE_MASK;
	buf->npages = last - first + 1;
	buf->pages = vmalloc(buf->npages * sizeof(buf->pages[0]));
	if (buf->pages == NULL)
		return -ENOMEM;
	memset(buf->pages, 0x00, buf->npages * sizeof(buf->pages[0]));

	down_read(&current->mm->mmap_sem);
	ret = get_user_pages(current, current->mm, data & PAGE_MASK,
			     buf->npages,
			     buf->vbuf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE, 0,
			     buf->pages, NULL);
	up_read(&current->mm->mmap_sem);

	if (ret != buf->npages) {
		isp_video_buffer_cleanup(buf);
		return -EFAULT;
	}

	ret = isp_video_buffer_lock_vma(buf, 1);
	if (ret < 0)
		isp_video_buffer_cleanup(buf);

	return ret;
}

static int isp_video_buffer_prepare_pfnmap(struct isp_video_buffer *buf)
{
	struct vm_area_struct *vma;
	unsigned long prev_pfn;
	unsigned long this_pfn;
	unsigned long start;
	unsigned long end;
	dma_addr_t pa;
	int ret = -EFAULT;


	start = buf->vbuf.m.userptr;
	end = buf->vbuf.m.userptr + buf->vbuf.length - 1;

	buf->offset = start & ~PAGE_MASK;
	buf->npages = (end >> PAGE_SHIFT) - (start >> PAGE_SHIFT) + 1;
	buf->pages = NULL;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, start);
	if (vma == NULL || vma->vm_end < end)
		goto done;

	for (prev_pfn = 0; start <= end; start += PAGE_SIZE) {
		ret = follow_pfn(vma, start, &this_pfn);
		if (ret)
			goto done;

		if (prev_pfn == 0)
			pa = this_pfn << PAGE_SHIFT;
		else if (this_pfn != prev_pfn + 1) {
			ret = -EFAULT;
			goto done;
		}

		prev_pfn = this_pfn;
	}

	buf->paddr = pa + buf->offset;
	ret = 0;

done:
	up_read(&current->mm->mmap_sem);
	return ret;
}

static int isp_video_buffer_prepare_vm_flags(struct isp_video_buffer *buf)
{
	struct vm_area_struct *vma;
	unsigned long start;
	unsigned long end;
	int ret = -EFAULT;

	start = buf->vbuf.m.userptr;
	end = buf->vbuf.m.userptr + buf->vbuf.length - 1;

	down_read(&current->mm->mmap_sem);

	do {
		vma = find_vma(current->mm, start);
		if (vma == NULL)
			goto done;

		if (start == buf->vbuf.m.userptr)
			buf->vm_flags = vma->vm_flags;

		if ((buf->vm_flags ^ vma->vm_flags) & VM_PFNMAP)
			goto done;

		start = vma->vm_end + 1;
	} while (vma->vm_end < end);

	ret = 0;

done:
	up_read(&current->mm->mmap_sem);
	return ret;
}

static int isp_video_buffer_prepare(struct isp_video_buffer *buf)
{
	int ret = -EINVAL;

	switch (buf->vbuf.memory) {
	case V4L2_MEMORY_MMAP:
		break;
	case V4L2_MEMORY_USERPTR:
		ret = isp_video_buffer_prepare_vm_flags(buf);
		if (ret < 0)
			return ret;

		if (buf->vm_flags & VM_PFNMAP) {
			ret =
				isp_video_buffer_prepare_pfnmap(buf);
			if (ret < 0)
				return ret;

		} else {
			ret = isp_video_buffer_prepare_user(buf);
			if (ret < 0)
				return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	if (buf->queue->ops->buffer_prepare)
		ret = buf->queue->ops->buffer_prepare(buf);

	if (ret < 0) {
		isp_video_buffer_cleanup(buf);
		return ret;
	}

	return ret;
}

static void isp_video_buffer_query(
					struct isp_video_buffer *buf,
					struct v4l2_buffer *vbuf)
{
	memcpy(vbuf, &buf->vbuf, sizeof(*vbuf));

	if (buf->vma_use_count)
		vbuf->flags |= V4L2_BUF_FLAG_MAPPED;

	switch (buf->state) {
	case ISP_BUF_STATE_ERROR:
		vbuf->flags |= V4L2_BUF_FLAG_ERROR;
	case ISP_BUF_STATE_DONE:
		vbuf->flags |= V4L2_BUF_FLAG_DONE;
	case ISP_BUF_STATE_QUEUED:
	case ISP_BUF_STATE_ACTIVE:
		vbuf->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case ISP_BUF_STATE_IDLE:
	default:
		break;
	}
}

static int isp_video_buffer_wait(struct isp_video_buffer *buf, int nonblocking)
{
	if (nonblocking) {
		return (buf->state != ISP_BUF_STATE_QUEUED &&
			buf->state != ISP_BUF_STATE_ACTIVE)
			? 0 : -EAGAIN;
	}

	return wait_event_interruptible(buf->wait,
		buf->state != ISP_BUF_STATE_QUEUED &&
		buf->state != ISP_BUF_STATE_ACTIVE);
}

static int isp_video_queue_free(struct isp_video_queue *queue)
{
	unsigned int i;
	struct isp_video_buffer *buf;

	if (queue->streaming)
		return -EBUSY;

	for (i = 0; i < queue->count; i++) {
		if (queue->buffers[i]->vma_use_count != 0)
			return -EBUSY;
	}

	for (i = 0; i < queue->count; i++) {
		buf = queue->buffers[i];

		isp_video_buffer_cleanup(buf);

		if (buf->vaddr != NULL) {
			vfree(buf->vaddr);
			buf->vaddr = NULL;
		}

		kfree(buf);
		queue->buffers[i] = NULL;
	}

	for (i = 0; i < ISP_VIDEO_MAX_DUMMY_BUFFERS; i++) {
		buf = queue->dummy_buffers[i];
		if (buf) {
			buf->dummy_pages = NULL;
			buf->vaddr = NULL;
			buf->paddr = 0;
			kfree(buf);
			queue->dummy_buffers[i] = NULL;
		}
	}

	INIT_LIST_HEAD(&queue->queue);
	queue->count = 0;

	return 0;
}

static int isp_video_queue_alloc(struct isp_video_queue *queue,
				 unsigned int nbuffers,
				 unsigned int size,
				 enum v4l2_memory memory, void *ext_dummy)
{
	struct isp_video_buffer *buf;
	unsigned int i;
	void *mem;
	int ret;

	/* Start by freeing the buffers. */
	ret = isp_video_queue_free(queue);
	if (ret < 0)
		return ret;

	/* Bail out of no buffers should be allocated. */
	if (nbuffers == 0)
		return 0;

	/* Initialize the allocated buffers. */
	for (i = 0; i < nbuffers; i++) {
		buf = kzalloc(queue->bufsize, GFP_KERNEL);
		if (buf == NULL)
			break;

		if (memory == V4L2_MEMORY_MMAP) {
			mem = vmalloc_32_user(PAGE_ALIGN(size));
			if (mem == NULL) {
				kfree(buf);
				break;
			}

			buf->vbuf.m.offset = i * PAGE_ALIGN(size);
			buf->vaddr = mem;
		} else {
			buf->vaddr = NULL;
		}

		buf->vbuf.index = i;
		buf->vbuf.length = size;
		buf->vbuf.type = queue->type;
		buf->vbuf.field = V4L2_FIELD_NONE;
		buf->vbuf.memory = memory;
		buf->vma_use_count = 0;

		init_waitqueue_head(&buf->wait);

		buf->queue = queue;

		queue->buffers[i] = buf;
		queue->count++;
	}

	if (i != nbuffers)
		return -ENOMEM;

	if (queue->enable_dummy == true) {
		/* Initialize the dummy buffers. */
		for (i = 0; i < ISP_VIDEO_MAX_DUMMY_BUFFERS; i++) {
			buf = kzalloc(queue->bufsize, GFP_KERNEL);
			if (buf == NULL)
				break;

			buf->order = get_order(PAGE_ALIGN(size));

			if (ext_dummy != NULL) {
				buf->vaddr = ext_dummy;
				buf->paddr = __pa(buf->vaddr);
			} else {
				buf->vaddr = NULL;
				buf->paddr = 0;
			}

			buf->queue = queue;
			buf->state = ISP_BUF_STATE_IDLE;
			queue->dummy_buffers[i] = buf;
		}

		if (i != ISP_VIDEO_MAX_DUMMY_BUFFERS)
			return -ENOMEM;
	}

	return 0;
}

int mvisp_video_queue_cleanup(struct isp_video_queue *queue)
{
	return isp_video_queue_free(queue);
}

int mvisp_video_queue_init(struct isp_video_queue *queue,
			      enum v4l2_buf_type type,
			      const struct isp_video_queue_operations *ops,
			      struct device *dev,
			      unsigned int bufsize, bool enable_dummy)
{
	INIT_LIST_HEAD(&queue->queue);
	mutex_init(&queue->lock);
	spin_lock_init(&queue->irqlock);

	queue->type = type;
	queue->ops = ops;
	queue->dev = dev;
	queue->bufsize = bufsize;
	queue->count = 0;
	queue->streaming = 0;
	queue->enable_dummy = enable_dummy;

	queue->poll_on_empty = false;
	init_waitqueue_head(&queue->empty_wait);

	memset(queue->buffers, 0x00,
			sizeof(struct isp_video_buffer *)
				* ISP_VIDEO_MAX_BUFFERS);
	memset(queue->dummy_buffers, 0x00,
			sizeof(struct isp_video_buffer *)
				* ISP_VIDEO_MAX_DUMMY_BUFFERS);

	return 0;
}

int mvisp_video_queue_reqbufs(struct isp_video_queue *queue,
				 struct v4l2_requestbuffers *rb,
				 void *ext_dummy)
{
	unsigned int nbuffers = rb->count;
	unsigned int size;
	int ret = 0;

	if (rb->type != queue->type)
		return -EINVAL;

	mutex_lock(&queue->lock);
	queue->ops->queue_prepare(queue, &nbuffers, &size);
	if (nbuffers == 0) {
		ret = isp_video_queue_free(queue);
		goto done;
	}
	if (size == 0) {
		ret = -EINVAL;
		goto done;
	}

	ret = isp_video_queue_alloc(queue, nbuffers,
				size, rb->memory, ext_dummy);
	if (ret < 0)
		goto done;

	ret = 0;

done:
	mutex_unlock(&queue->lock);
	return ret;
}

int mvisp_video_queue_querybuf(struct isp_video_queue *queue,
				  struct v4l2_buffer *vbuf)
{
	struct isp_video_buffer *buf;
	int ret = 0;

	if (vbuf->type != queue->type)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (vbuf->index >= queue->count) {
		ret = -EINVAL;
		goto done;
	}

	buf = queue->buffers[vbuf->index];
	isp_video_buffer_query(buf, vbuf);

done:
	mutex_unlock(&queue->lock);
	return ret;
}

int mvisp_video_queue_qbuf(struct isp_video_queue *queue,
			      struct v4l2_buffer *vbuf)
{
	struct isp_video_buffer *buf;
	unsigned long flags;
	int ret = -EINVAL;
	unsigned int empty;

	if (vbuf->type != queue->type)
		goto done;

	mutex_lock(&queue->lock);

	if (vbuf->index >= queue->count)
		goto done;

	buf = queue->buffers[vbuf->index];

	if (vbuf->memory != buf->vbuf.memory)
		goto done;

	if (buf->state != ISP_BUF_STATE_IDLE)
		goto done;

	if (vbuf->memory == V4L2_MEMORY_USERPTR &&
	    vbuf->m.userptr != buf->vbuf.m.userptr) {
		isp_video_buffer_cleanup(buf);
		buf->vbuf.m.userptr = vbuf->m.userptr;
		buf->prepared = 0;
	}

	if (!buf->prepared) {
		ret = isp_video_buffer_prepare(buf);
		if (ret < 0)
			goto done;
		buf->prepared = 1;
	}

	empty = list_empty(&queue->queue);
	buf->state = ISP_BUF_STATE_QUEUED;
	list_add_tail(&buf->stream, &queue->queue);
	memcpy(&buf->vbuf, vbuf, sizeof(*vbuf));

	if (queue->streaming) {
		spin_lock_irqsave(&queue->irqlock, flags);
		queue->ops->buffer_queue(buf);
		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

	if (empty)
		wake_up(&queue->empty_wait);

	ret = 0;

done:
	mutex_unlock(&queue->lock);
	return ret;
}

int mvisp_video_queue_dqbuf(struct isp_video_queue *queue,
			       struct v4l2_buffer *vbuf, int nonblocking)
{
	struct isp_video_buffer *buf;
	int ret;

	if (vbuf->type != queue->type)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (list_empty(&queue->queue)) {
		ret = -EINVAL;
		goto done;
	}

	buf = list_first_entry(&queue->queue, struct isp_video_buffer, stream);
	ret = isp_video_buffer_wait(buf, nonblocking);
	if (ret < 0)
		goto done;

	list_del(&buf->stream);

	isp_video_buffer_query(buf, vbuf);
	buf->state = ISP_BUF_STATE_IDLE;
	vbuf->flags &= ~V4L2_BUF_FLAG_QUEUED;

done:
	mutex_unlock(&queue->lock);
	return ret;
}

int mvisp_video_queue_streamon(struct isp_video_queue *queue)
{
	struct isp_video_buffer *buf;
	unsigned long flags;

	mutex_lock(&queue->lock);

	if (queue->streaming)
		goto done;

	queue->streaming = 1;

	spin_lock_irqsave(&queue->irqlock, flags);
	list_for_each_entry(buf, &queue->queue, stream)
		queue->ops->buffer_queue(buf);
	spin_unlock_irqrestore(&queue->irqlock, flags);

done:
	mutex_unlock(&queue->lock);
	return 0;
}

void mvisp_video_queue_streamoff(struct isp_video_queue *queue)
{
	struct isp_video_buffer *buf;
	unsigned long flags;
	unsigned int i;

	mutex_lock(&queue->lock);

	if (!queue->streaming)
		goto done;

	queue->streaming = 0;

	spin_lock_irqsave(&queue->irqlock, flags);
	for (i = 0; i < queue->count; ++i) {
		buf = queue->buffers[i];

		if (buf->state == ISP_BUF_STATE_ACTIVE)
			wake_up(&buf->wait);

		buf->state = ISP_BUF_STATE_IDLE;
	}

	for (i = 0; i < ISP_VIDEO_MAX_DUMMY_BUFFERS; ++i) {
		buf = queue->dummy_buffers[i];
		if (buf != NULL)
			buf->state = ISP_BUF_STATE_IDLE;
	}
	spin_unlock_irqrestore(&queue->irqlock, flags);

	INIT_LIST_HEAD(&queue->queue);

done:
	mutex_unlock(&queue->lock);
}

static void isp_video_queue_vm_open(struct vm_area_struct *vma)
{
	struct isp_video_buffer *buf = vma->vm_private_data;

	buf->vma_use_count++;
}

static void isp_video_queue_vm_close(struct vm_area_struct *vma)
{
	struct isp_video_buffer *buf = vma->vm_private_data;

	buf->vma_use_count--;
}

static const struct vm_operations_struct isp_video_queue_vm_ops = {
	.open = isp_video_queue_vm_open,
	.close = isp_video_queue_vm_close,
};

int mvisp_video_queue_mmap(struct isp_video_queue *queue,
			 struct vm_area_struct *vma)
{
	struct isp_video_buffer *uninitialized_var(buf);
	unsigned long size;
	unsigned int i;
	int ret = 0;

	mutex_lock(&queue->lock);

	for (i = 0; i < queue->count; ++i) {
		buf = queue->buffers[i];
		if ((buf->vbuf.m.offset >> PAGE_SHIFT) == vma->vm_pgoff)
			break;
	}

	if (i == queue->count) {
		ret = -EINVAL;
		goto done;
	}

	size = vma->vm_end - vma->vm_start;

	if (buf->vbuf.memory != V4L2_MEMORY_MMAP ||
	    size != PAGE_ALIGN(buf->vbuf.length)) {
		ret = -EINVAL;
		goto done;
	}

	ret = remap_vmalloc_range(vma, buf->vaddr, 0);
	if (ret < 0)
		goto done;

	vma->vm_ops = &isp_video_queue_vm_ops;
	vma->vm_private_data = buf;
	isp_video_queue_vm_open(vma);

done:
	mutex_unlock(&queue->lock);
	return ret;
}

unsigned int mvisp_video_queue_poll(struct isp_video_queue *queue,
				       struct file *file, poll_table *wait)
{
	struct isp_video_buffer *buf;
	unsigned int mask = 0;

	mutex_lock(&queue->lock);
	if (list_empty(&queue->queue)) {
		if (queue->poll_on_empty == false)
			poll_wait(file, &queue->empty_wait, wait);

		mask = 0;
		queue->poll_on_empty = true;
		goto done;
	}
	buf = list_first_entry(&queue->queue, struct isp_video_buffer, stream);

	poll_wait(file, &buf->wait, wait);
	queue->poll_on_empty = false;
	if (buf->state == ISP_BUF_STATE_DONE ||
	    buf->state == ISP_BUF_STATE_ERROR) {
		if (queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			mask |= POLLIN | POLLRDNORM;
		else
			mask |= POLLOUT | POLLWRNORM;
	}

done:
	mutex_unlock(&queue->lock);
	return mask;
}
