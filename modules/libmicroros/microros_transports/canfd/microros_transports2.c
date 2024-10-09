#include <uxr/client/transport.h>

#include <version.h>
#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(3,1,0)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/posix/unistd.h>
#else 
#include <zephyr.h>
#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <usb/usb_device.h>
#include <posix/unistd.h>
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <microros_transports.h>


#define RING_BUF_SIZE 2048

char uart_in_buffer[RING_BUF_SIZE];
char uart_out_buffer[RING_BUF_SIZE];

struct ring_buf out_ringbuf, in_ringbuf;

static void uart_fifo_callback(const struct device *dev, void * user_data){
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            int recv_len;
            char buffer[64];
            size_t len = MIN(ring_buf_space_get(&in_ringbuf), sizeof(buffer));

            if (len > 0){
                recv_len = uart_fifo_read(dev, buffer, len);
                ring_buf_put(&in_ringbuf, buffer, recv_len);
            }

        }

        if (uart_irq_tx_ready(dev)) {
            char buffer[64];
            int rb_len;

            rb_len = ring_buf_get(&out_ringbuf, buffer, sizeof(buffer));

            if (rb_len == 0) {
                uart_irq_tx_disable(dev);
                continue;
            }

            uart_fifo_fill(dev, buffer, rb_len);
        }
    }
}

bool zephyr_transport_open(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    const struct device *dev = DEVICE_DT_GET(CANBUS_NODE);
	struct k_sem tx_queue_sem;
	struct can_frame frame = {0};
	int err;

	k_sem_init(&tx_queue_sem, CONFIG_SAMPLE_CAN_BABBLING_TX_QUEUE_SIZE,
		   CONFIG_SAMPLE_CAN_BABBLING_TX_QUEUE_SIZE);

	if (!device_is_ready(dev)) {
		printk("CAN device not ready");
		return 0;
	}

	if (IS_ENABLED(CONFIG_SAMPLE_CAN_BABBLING_FD_MODE)) {
		err = can_set_mode(dev, CAN_MODE_FD);
		if (err != 0) {
			printk("Error setting CAN FD mode (err %d)", err);
			return 0;
		}
	}

	err = can_start(dev);
	if (err != 0) {
		printk("Error starting CAN controller (err %d)", err);
		return 0;
	}
    else{
        printk("CAN controller started");
    }


    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    (void) params;

    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t wrote;

    wrote = ring_buf_put(&out_ringbuf, buf, len);

    uart_irq_tx_enable(params->uart_dev);

    while (!ring_buf_is_empty(&out_ringbuf)){
        k_sleep(K_MSEC(5));
    }

    return wrote;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    size_t read = 0;
    int spent_time = 0;

    while(ring_buf_is_empty(&in_ringbuf) && spent_time < timeout){
        k_sleep(K_MSEC(1));
        spent_time++;
    }

    uart_irq_rx_disable(params->uart_dev);
    read = ring_buf_get(&in_ringbuf, buf, len);
    uart_irq_rx_enable(params->uart_dev);

    return read;
}