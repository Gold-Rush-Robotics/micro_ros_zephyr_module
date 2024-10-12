/* Devicetree */
#include <microros_transports.h>

bool zephyr_transport_open(struct uxrCustomTransport * transport){
	zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
	struct k_sem tx_queue_sem;
	// struct can_frame frame = {0};
	int err;
    params->dev = DEVICE_DT_GET(CANBUS_NODE);

	k_sem_init(&tx_queue_sem, CONFIG_CAN_TX_QUEUE_SIZE,
		   CONFIG_CAN_TX_QUEUE_SIZE);

	if (!device_is_ready(params->dev)) {
		return 0;
	}

    err = can_set_mode(params->dev, true);
    if (err != 0) {
        return 0;
    }
    err = can_set_bitrate(params->dev, CONFIG_CANFD_BITRATE);
    if (err != 0) {
        return 0;
    }
    // err = can_set_bitrate_data(params->dev, CONFIG_CANFD_BITRATE_DATA);
    // if (err != 0) {
    //     printk("Error setting CAN FD bitrate (err %d)", err);
    //     return 0;
    // }

	err = can_start(params->dev);
	if (err != 0) {
		return 0;
	}
    else{
    }


    return true;
    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    // TODO: close serial transport here
    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    struct can_frame frame = {
        .flags = 0,
        .id = 0x123,
        .dlc = 8,
        .data = {1,2,3,4,5,6,7,8}
    };
    int ret;
    ret = can_send(params->dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
        return 0;
    }
    return 0;

}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;

    return 0;
}