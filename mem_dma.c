#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "mem_dma"
#define CLASS_NAME "dma_class"
#define DEVICE_NAME "mem_dma_test_dev"
#define DMA_BUFFER_SIZE (1 * 1024 * 1024 * 1024) // 1GB buffer

#define DMA0_BAR0_OFFSET 0x1000
#define HOST_DMA_BASE_ADDR 0x0000000101500000
#define DMA_CH_SIZE_0 (1 * 1024 * 1024)    // 1 MB
#define DMA_CH_SIZE_32 (32 * 1024 * 1024)  // 32 MB
#define DMA_CH_SIZE_33 (64 * 1024 * 1024)  // 64 MB
#define DMA_CH_SIZE_34 (128 * 1024 * 1024) // 128 MB
#define DMA_CH_SIZE_35 (256 * 1024 * 1024) // 256 MB

/**
 * struct xilinx_dma_dev - 私有数据结构，用于存储每个设备的状态
 * @pdev: 指向此驱动实例管理的 pci_dev 结构体的指针
 * @dma_virt_addr: DMA 缓冲区的内核虚拟地址 (CPU 使用)
 * @dma_handle: DMA 缓冲区的总线地址 (设备使用)
 * @monitor_task: 指向监控内核线程的 task_struct 结构体的指针
 */
typedef struct {
  struct pci_dev *pdev;
  void *dma_virt_addr;
  void __iomem *bar0_virt_addr;
  dma_addr_t dma_handle;
  struct task_struct *monitor_task;
} xilinx_dma_dev_t;

static const struct pci_device_id xilinx_pci_ids[] = {
    {PCI_DEVICE(0x10ee, 0x903f)},
    {PCI_DEVICE(0x10ee, 0x9038)},
    {
        0,
    }};
#ifdef ARM_BUILD
static void write_64_to_bar0(void __iomem *bar0, u64 addr, u64 dma_ch_num) {
  u32 addr_low = lower_32_bits(addr);
  u32 addr_high = upper_32_bits(addr);
  iowrite32(addr_low, bar0 + 0x2200000 + DMA0_BAR0_OFFSET + dma_ch_num * 8);
  iowrite32(addr_high,
            bar0 + 0x2200000 + DMA0_BAR0_OFFSET + dma_ch_num * 8 + 4);
}
#endif

#ifdef DEBUG
static int monitor_thread(void *data) {
  xilinx_dma_dev_t *dev_priv = (xilinx_dma_dev_t *)data;
  u32 __iomem *dma_buffer_ptr = (u32 __iomem *)dev_priv->dma_virt_addr;

  pr_info("%s: Monitor thread started for device %s\n", DRIVER_NAME,
          pci_name(dev_priv->pdev));

  allow_signal(SIGKILL);

  while (!kthread_should_stop()) {
    u32 value;

    value = readl(dma_buffer_ptr);

    pr_info("%s: First 4 bytes of DMA buffer: 0x%08x\n", DRIVER_NAME, value);

    msleep(1000);

    if (signal_pending(current)) {
      break;
    }
  }

  pr_info("%s: Monitor thread for device %s stopping\n", DRIVER_NAME,
          pci_name(dev_priv->pdev));
  dev_priv->monitor_task = NULL;
  return 0;
}
#endif

static int mem_dma_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
  xilinx_dma_dev_t *dev_priv;
  int err, i;
  u64 host_base_addr;
  phys_addr_t phys_addr;

  pr_info("%s: Probing for device %s\n", DRIVER_NAME, pci_name(pdev));

  dev_priv = kzalloc(sizeof(xilinx_dma_dev_t), GFP_KERNEL);
  if (!dev_priv) {
    pr_err("%s: Failed to allocate private data structure\n", DRIVER_NAME);
    return -ENOMEM;
  }
  dev_priv->pdev = pdev;
  pci_set_drvdata(pdev, dev_priv);

  err = pci_enable_device(pdev);
  if (err) {
    pr_err("%s: Failed to enable PCI device\n", DRIVER_NAME);
    goto err_free_priv;
  }

  err = pci_request_regions(pdev, DRIVER_NAME);
  if (err) {
    pr_err("%s: Failed to request PCI regions\n", DRIVER_NAME);
    goto err_disable_device;
  }

  pci_set_master(pdev);

  dev_priv->bar0_virt_addr = pci_iomap(pdev, 0, 0);
  if (!dev_priv->bar0_virt_addr) {
    pr_err("%s: iomap BAR0 failed.\n", DRIVER_NAME);
    err = -EIO;
    goto err_release_regions;
  }

  err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
  if (err) {
    pr_warn("%s: 64-bit DMA not supported, trying 32-bit\n", DRIVER_NAME);
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (err) {
      pr_err("%s: No suitable DMA mask available\n", DRIVER_NAME);
      goto err_release_regions;
    }
  }

  dev_priv->dma_virt_addr = dma_alloc_coherent(
      &pdev->dev, DMA_BUFFER_SIZE, &dev_priv->dma_handle, GFP_KERNEL);
  if (!dev_priv->dma_virt_addr) {
    pr_err("%s: Failed to allocate DMA coherent buffer\n", DRIVER_NAME);
    err = -ENOMEM;
    goto err_release_regions;
  }
  pr_info(
      "%s: DMA buffer allocated. CPU virt_addr: %p, Device dma_handle: %pad\n",
      DRIVER_NAME, dev_priv->dma_virt_addr, &dev_priv->dma_handle);

  phys_addr = virt_to_phys(dev_priv->dma_virt_addr);
  pr_info("%s: DMA buffer allocated:\n", DRIVER_NAME);
  pr_info("%s:  -> CPU virtual address : %p\n", DRIVER_NAME,
          dev_priv->dma_virt_addr);
  pr_info("%s:  -> Physical address    : %pa\n", DRIVER_NAME, &phys_addr);
  pr_info("%s:  -> Device bus address  : %pad\n", DRIVER_NAME,
          &dev_priv->dma_handle);

#ifdef ARM_BUILD
  // channel 0-31
  for (i = 0; i < 32; i++) {
    write_64_to_bar0(dev_priv->bar0_virt_addr,
                     HOST_DMA_BASE_ADDR + DMA_CH_SIZE_0 * i, i);
  }
  host_base_addr = HOST_DMA_BASE_ADDR + DMA_CH_SIZE_0 * 31;
  write_64_to_bar0(dev_priv->bar0_virt_addr, host_base_addr + DMA_CH_SIZE_32,
                   32);
  host_base_addr += DMA_CH_SIZE_32;
  write_64_to_bar0(dev_priv->bar0_virt_addr, host_base_addr + DMA_CH_SIZE_33,
                   33);
  host_base_addr += DMA_CH_SIZE_33;
  write_64_to_bar0(dev_priv->bar0_virt_addr, host_base_addr + DMA_CH_SIZE_34,
                   34);
  host_base_addr += DMA_CH_SIZE_34;
  write_64_to_bar0(dev_priv->bar0_virt_addr, host_base_addr + DMA_CH_SIZE_35,
                   35);
#endif

  writel(0x66666666, (void __iomem *)dev_priv->dma_virt_addr);

#ifdef DEBUG
  dev_priv->monitor_task =
      kthread_run(monitor_thread, dev_priv, "%s_mon", DEVICE_NAME);
  if (IS_ERR(dev_priv->monitor_task)) {
    pr_err("%s: Failed to create and run kthread\n", DRIVER_NAME);
    err = PTR_ERR(dev_priv->monitor_task);
    goto err_free_dma;
  }
#endif

  pr_info("%s: Device %s probed successfully\n", DRIVER_NAME, pci_name(pdev));
  return 0;

err_free_dma:
  dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE, dev_priv->dma_virt_addr,
                    dev_priv->dma_handle);
err_release_regions:
  pci_clear_master(pdev);
  pci_release_regions(pdev);
err_disable_device:
  pci_disable_device(pdev);
err_free_priv:
  kfree(dev_priv);
  pci_set_drvdata(pdev, NULL);
  return err;
}

static void mem_dma_remove(struct pci_dev *pdev) {
  xilinx_dma_dev_t *dev_priv = pci_get_drvdata(pdev);

  pr_info("%s: Removing device %s\n", DRIVER_NAME, pci_name(pdev));

  if (!dev_priv) {
    return;
  }

  if (dev_priv->monitor_task) {
    kthread_stop(dev_priv->monitor_task);
  }

  if (dev_priv->dma_virt_addr) {
    dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE, dev_priv->dma_virt_addr,
                      dev_priv->dma_handle);
  }

  pci_clear_master(pdev);

  pci_release_regions(pdev);

  pci_disable_device(pdev);

  kfree(dev_priv);
  pci_set_drvdata(pdev, NULL);
}

MODULE_DEVICE_TABLE(pci, xilinx_pci_ids);

static struct pci_driver mem_dma_driver = {
    .name = "mem-dma",
    .id_table = xilinx_pci_ids,
    .probe = mem_dma_probe,
    .remove = mem_dma_remove,
};

static int __init mem_dma_init_module(void) {
  pr_info("%s: Loading module...\n", DRIVER_NAME);
  return pci_register_driver(&mem_dma_driver);
}

static void __exit mem_dma_exit_module(void) {
  pr_info("%s: Unloading module...\n", DRIVER_NAME);
  pci_unregister_driver(&mem_dma_driver);
}

module_init(mem_dma_init_module);
module_exit(mem_dma_exit_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ZQY");
MODULE_DESCRIPTION("A driver to allocate mem for dma.");