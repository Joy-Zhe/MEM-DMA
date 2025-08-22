#include <linux/gfp.h>
#include <linux/ktime.h>
#include <linux/printk.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/proc_fs.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "mem_dma"
#define CLASS_NAME "dma_class"
#define DEVICE_NAME "mem_dma_test_dev"
#define DMA_BUFFER_SIZE (1 * 1024 * 1024 * 1024) // 1GB buffer
#define DMA_BUFFER_SIZE_ARM (1 * 1024 * 1024)

#define DMA0_BAR0_OFFSET 0x1000
#define HOST_DMA_BASE_ADDR 0x0000000101500000
#define DMA_CH_SIZE_0 (1 * 1024 * 1024)    // 1 MB
#define DMA_CH_SIZE_32 (32 * 1024 * 1024)  // 32 MB
#define DMA_CH_SIZE_33 (64 * 1024 * 1024)  // 64 MB
#define DMA_CH_SIZE_34 (128 * 1024 * 1024) // 128 MB
#define DMA_CH_SIZE_35 (256 * 1024 * 1024) // 256 MB

// #define BAR4_TEST
#define BAR4_TEST_SIZE (1 * 1024 * 1024)
#define BAR4_TEST_ITERATION (1000) 
#define BAR4_LATENCY_ITERATION (10000)
#define BAR4_TEST_OFFSET (0x0)

/**
 * struct bar4_perf_stats - BAR4性能统计数据
 * @read_bandwidth_mbps: 读取带宽 (MB/s)
 * @write_bandwidth_mbps: 写入带宽 (MB/s)
 * @read_latency_ns: 平均读取延迟 (ns)
 * @write_latency_ns: 平均写入延迟 (ns)
 * @min_read_latency_ns: 最小读取延迟 (ns)
 * @max_read_latency_ns: 最大读取延迟 (ns)
 * @min_write_latency_ns: 最小写入延迟 (ns)
 * @max_write_latency_ns: 最大写入延迟 (ns)
 */
typedef struct {
  u64 read_bandwidth_mbps;
  u64 write_bandwidth_mbps;
  u64 read_latency_ns;
  u64 write_latency_ns;
  u64 min_read_latency_ns;
  u64 max_read_latency_ns;
  u64 min_write_latency_ns;
  u64 max_write_latency_ns;
} bar4_perf_stats_t;

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
#ifdef BAR4_TEST
  void __iomem *bar4_virt_addr;
  bar4_perf_stats_t perf_stats;
  // u8 *test_buffer;
  resource_size_t bar4_size;
#endif
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

#ifdef BAR4_TEST
static u64 bar4_performance_test(xilinx_dma_dev_t *dev_priv, char rw, char type,
                                 u64 *min_latency, u64 *max_latency) {
  ktime_t start_time, end_time;
  u64 total_bytes = (u64)BAR4_TEST_SIZE * BAR4_TEST_ITERATION;
  u64 duration_ns, bandwidth_mbps, total_latency = 0, latency_ns,
                                   min_lat = U64_MAX, max_lat = 0, avg_latency;
  int i, j;
  void __iomem *bar4_ptr;
  u32 *test_data;

  if (!dev_priv->bar4_virt_addr) {
    pr_err("%s: BAR4 not mapped.\n", DRIVER_NAME);
    return 0;
  }

  // BAR4 size check
  if (type == 'b' &&
      dev_priv->bar4_size < (BAR4_TEST_OFFSET + BAR4_TEST_SIZE)) {
    pr_err("%s: BAR4 size (%llu) too small for bandwidth test\n", DRIVER_NAME,
           (u64)dev_priv->bar4_size);
    return 0;
  }

  if (type == 'l' && dev_priv->bar4_size < (BAR4_TEST_OFFSET + sizeof(u32))) {
    pr_err("%s: BAR4 size (%llu) too small for latency test\n", DRIVER_NAME,
           (u64)dev_priv->bar4_size);
    return 0;
  }

  // BAR4 start test offset
  bar4_ptr = dev_priv->bar4_virt_addr + BAR4_TEST_OFFSET;

  test_data = kmalloc(BAR4_TEST_SIZE, GFP_KERNEL);
  if (!test_data) {
    pr_err("%s: Failed to allocate test data buffer\n", DRIVER_NAME);
    return 0;
  }

  // fill
  for (i = 0; i < BAR4_TEST_SIZE / sizeof(u32); i++) {
    test_data[i] = 0x12345678 + i;
  }

  // Bandwidth Test
  if (type == 'b') {
    // prepare
    for (i = 0; i < 10; i++) {
      if (rw == 'w') {
        memcpy_toio(bar4_ptr, test_data, BAR4_TEST_SIZE);
      } else if (rw == 'r') {
        memcpy_fromio(test_data, bar4_ptr, BAR4_TEST_SIZE);
      }
    }

    start_time = ktime_get();

    // Bandwidth
    for (i = 0; i < BAR4_TEST_ITERATION; i++) {
      if (rw == 'w') {
        memcpy_toio(bar4_ptr, test_data, BAR4_TEST_SIZE);
      } else if (rw == 'r') {
        memcpy_fromio(test_data, bar4_ptr, BAR4_TEST_SIZE);
      }
    }

    end_time = ktime_get();

    duration_ns = ktime_to_ns(ktime_sub(end_time, start_time));
    if (duration_ns == 0) {
      pr_warn("%s: %s test duration too short to measure\n", DRIVER_NAME,
              (rw == 'w') ? "Write" : "Read");
      kfree(test_data);
      return 0;
    }

    bandwidth_mbps =
        (total_bytes * 1000) / (duration_ns * 1024 * 1024 / 1000000);

    // pr_info("%s: BAR4 %s bandwidth test: %llu MB/s\n", DRIVER_NAME,
    //         (rw == 'w') ? "Write" : "Read", bandwidth_mbps);

    kfree(test_data);
    return bandwidth_mbps;
  }
  // Latency Test
  else if (type == 'l') {
    // 预热
    for (i = 0; i < 100; i++) {
      if (rw == 'w')
        memcpy_toio(bar4_ptr, test_data, BAR4_TEST_SIZE);
      else
        memcpy_fromio(test_data, bar4_ptr, BAR4_TEST_SIZE);
    }

    for (i = 0; i < BAR4_TEST_ITERATION; i++) {
      start_time = ktime_get();
      if (rw == 'w')
        memcpy_toio(bar4_ptr, test_data, BAR4_TEST_SIZE);
      else
        memcpy_fromio(test_data, bar4_ptr, BAR4_TEST_SIZE);
      end_time = ktime_get();

      latency_ns = ktime_to_ns(ktime_sub(end_time, start_time));
      total_latency += latency_ns;

      if (latency_ns < min_lat)
        min_lat = latency_ns;
      if (latency_ns > max_lat)
        max_lat = latency_ns;
    }

    *min_latency = min_lat;
    *max_latency = max_lat;

    avg_latency = total_latency / BAR4_TEST_ITERATION;
    // printk("[DEBUG]: total_latency = %llu", total_latency);

    // pr_info("%s: BAR4 %s latency - Avg: %llu ns, Min: %llu ns, Max: %llu
    // ns\n",
    //         DRIVER_NAME, (rw == 'w') ? "Write" : "Read", avg_latency,
    //         min_lat, max_lat);

    kfree(test_data);
    return avg_latency;
  }
  // Invalid Test
  else {
    pr_err("%s: Invalid test type '%c'\n", DRIVER_NAME, type);
    kfree(test_data);
    return 0;
  }
}

static void run_bar4_tests(xilinx_dma_dev_t *dev_priv) {
  pr_info("%s: Starting BAR4 performance tests...\n", DRIVER_NAME);

  if (!dev_priv->bar4_virt_addr) {
    pr_err("%s: BAR4 not available for testing\n", DRIVER_NAME);
    return;
  }

  dev_priv->perf_stats.write_bandwidth_mbps =
      bar4_performance_test(dev_priv, 'w', 'b', NULL, NULL);
  dev_priv->perf_stats.read_bandwidth_mbps =
      bar4_performance_test(dev_priv, 'r', 'b', NULL, NULL);
  dev_priv->perf_stats.write_latency_ns = bar4_performance_test(
      dev_priv, 'w', 'l', &dev_priv->perf_stats.min_write_latency_ns,
      &dev_priv->perf_stats.max_write_latency_ns);
  dev_priv->perf_stats.read_latency_ns = bar4_performance_test(
      dev_priv, 'r', 'l', &dev_priv->perf_stats.min_read_latency_ns,
      &dev_priv->perf_stats.max_read_latency_ns);
  pr_info("%s: BAR4 Performance Test Summary:\n", DRIVER_NAME);
  pr_info("%s: ================================\n", DRIVER_NAME);
  pr_info("%s: Write Bandwidth: %llu MB/s\n", DRIVER_NAME,
          dev_priv->perf_stats.write_bandwidth_mbps);
  pr_info("%s: Read Bandwidth:  %llu MB/s\n", DRIVER_NAME,
          dev_priv->perf_stats.read_bandwidth_mbps);
  pr_info("%s: Write Latency:   %llu ns (Min: %llu, Max: %llu)\n", DRIVER_NAME,
          dev_priv->perf_stats.write_latency_ns,
          dev_priv->perf_stats.min_write_latency_ns,
          dev_priv->perf_stats.max_write_latency_ns);
  pr_info("%s: Read Latency:    %llu ns (Min: %llu, Max: %llu)\n", DRIVER_NAME,
          dev_priv->perf_stats.read_latency_ns,
          dev_priv->perf_stats.min_read_latency_ns,
          dev_priv->perf_stats.max_read_latency_ns);
  pr_info("%s: ================================\n", DRIVER_NAME);
}
#endif

static int mem_dma_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
  xilinx_dma_dev_t *dev_priv;
  int err, i;
  u64 host_base_addr;
  phys_addr_t phys_addr;
  u8 pci_cmd;

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

  // setpci implementation
#ifdef HOST_BUILD
  pci_read_config_byte(pdev, PCI_COMMAND, &pci_cmd);
  pr_info("%s: Original PCI CMD=0x%02x\n", DRIVER_NAME, pci_cmd);
  pci_cmd = 0x07; // == setpci -s 5e:00.0 0x4.b=0x07
  pci_write_config_byte(pdev, PCI_COMMAND, pci_cmd);
  pr_info("%s: HOST build -> Set PCI CMD=0x%02x\n", DRIVER_NAME, pci_cmd);
#endif

#ifdef ARM_BUILD
  pci_read_config_byte(pdev, PCI_COMMAND, &pci_cmd);
  pr_info("%s: Original PCI CMD=0x%02x\n", DRIVER_NAME, pci_cmd);
  pci_cmd = 0x06; // == setpci -s 01:00.0 0x4.b=0x06
  pci_write_config_byte(pdev, PCI_COMMAND, pci_cmd);
  pr_info("%s: ARM build -> Set PCI CMD=0x%02x\n", DRIVER_NAME, pci_cmd);
#endif

  // BAR0 mapping
  dev_priv->bar0_virt_addr = pci_iomap(pdev, 0, 0);
  if (!dev_priv->bar0_virt_addr) {
    pr_err("%s: iomap BAR0 failed.\n", DRIVER_NAME);
    err = -EIO;
    goto err_release_regions;
  }
#ifdef BAR4_TEST
  // BAR4 mapping
  dev_priv->bar4_size = pci_resource_len(pdev, 4);
  if (dev_priv->bar4_size > 0) {
    dev_priv->bar4_virt_addr = pci_iomap(pdev, 4, 0);
    if (!dev_priv->bar4_virt_addr) {
      pr_warn("%s: iomap BAR4 failed, BAR4 tests will be skipped\n",
              DRIVER_NAME);
    } else {
      pr_info("%s: BAR4 mapped successfully, size: %llu bytes\n", DRIVER_NAME,
              (u64)dev_priv->bar4_size);
    }
  } else {
    pr_warn("%s: BAR4 not present or has zero size\n", DRIVER_NAME);
  }

#endif

  err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
  if (err) {
    pr_warn("%s: 64-bit DMA not supported, trying 32-bit\n", DRIVER_NAME);
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (err) {
      pr_err("%s: No suitable DMA mask available\n", DRIVER_NAME);
      goto err_release_regions;
    }
  }
#ifdef ARM_BUILD
  dev_priv->dma_virt_addr = dma_alloc_coherent(
      &pdev->dev, DMA_BUFFER_SIZE_ARM, &dev_priv->dma_handle, GFP_KERNEL);
#else
  dev_priv->dma_virt_addr = dma_alloc_coherent(
      &pdev->dev, DMA_BUFFER_SIZE, &dev_priv->dma_handle, GFP_KERNEL);
#endif
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

#ifdef BAR4_TEST
  if (dev_priv->bar4_virt_addr) {
    run_bar4_tests(dev_priv);
    // kfree(dev_priv->test_buffer);
  }
#endif

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
#ifdef ARM_BUILD
    dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE_ARM, dev_priv->dma_virt_addr,
                      dev_priv->dma_handle);
#else
    dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE, dev_priv->dma_virt_addr,
                      dev_priv->dma_handle);
#endif
  }

  if (dev_priv->bar0_virt_addr) {
    pci_iounmap(pdev, dev_priv->bar0_virt_addr);
  }
#ifdef BAR4_TEST
  if (dev_priv->bar4_virt_addr) {
    pci_iounmap(pdev, dev_priv->bar4_virt_addr);
  }
#endif

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