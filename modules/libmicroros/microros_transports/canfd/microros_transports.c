/* Devicetree */
#include <microros_transports.h>
#include <stdio.h>

bool zephyr_transport_open(struct uxrCustomTransport * transport){
	zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
	struct k_sem tx_queue_sem;
	// struct can_frame frame = {0};
	int err;
    params->dev = DEVICE_DT_GET(CANBUS_NODE);

	k_sem_init(&tx_queue_sem, CONFIG_CAN_TX_QUEUE_SIZE,
		   CONFIG_CAN_TX_QUEUE_SIZE);
    printf("Set speed");

	if (!device_is_ready(params->dev)) {
        printf("device is not ready\n");
		return 0;
	}
    else{
        printf("device ready\n");
    }

    err = can_set_mode(params->dev, true);
    if (err != 0) {
        printf("can mode not set\n");
        return 0;
    }
    err = can_set_bitrate(params->dev, CONFIG_CANFD_BITRATE);
    if (err != 0) {
        printf("can bitrate not set\n");
        return 0;
    }
    err = can_set_bitrate_data(params->dev, CONFIG_CANFD_BITRATE_DATA);
    if (err != 0) {
        printf("Error setting CAN FD bitrate (err %d) \n", err);
        return 0;
    }

	err = can_start(params->dev);
	if (err != 0) {
        printf("can not started \n");
		return 0;
	}
    else{
        printf("can started \n");
    }
    printf("can started \n");
    return true;
}

bool zephyr_transport_close(struct uxrCustomTransport * transport){
    (void) transport;
    printf("Attmepting to close \n");
    // TODO: close serial transport here
    return true;
}

size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    printf("Attmepting to send \n");
    zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    struct can_frame frame = {
        .flags = 0,
        .id = 0x123,
        .dlc = len < 8 ? len : 8 // CAN frame data length code (DLC) is max 8
    };
    memcpy(frame.data, buf, frame.dlc);

    int ret = can_send(params->dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
        if (err) {
            *err = ret;
        }
        return 0;
    }
    return frame.dlc;
}

size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    printf("Attmepting to read \n");
    // zephyr_transport_params_t * params = (zephyr_transport_params_t*) transport->args;
    // struct can_frame frame;
    // int ret = can_recv(params->dev, &frame, K_MSEC(timeout), NULL);

    // if (ret != 0) {
    //     if (err) {
    //         *err = ret;
    //     }
    //     return 0;
    // }

    // size_t bytes_to_copy = len < frame.dlc ? len : frame.dlc;
    // memcpy(buf, frame.data, bytes_to_copy);
    return 8;
}