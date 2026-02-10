#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <mosquitto.h>
#include <cjson/cJSON.h>

// --- Configuration Structure ---
typedef struct {
    char serial_port[64];
    int baud_rate;
    char mqtt_broker[64];
    int mqtt_port;
    char mqtt_user[64];
    char mqtt_password[64];
    int poll_interval;
    char device_name[64];
    char device_id[64];
} AppConfig;

AppConfig config;

// --- Serial Helpers ---
int setup_serial(const char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no flow control

    // Raw mode
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = ~(IXON | IXOFF | IXANY);

    // Timeout settings (crucial for RS485)
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 10;           // 1.0 seconds read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return fd;
}

// --- JBD Protocol ---
uint16_t calculate_checksum(uint8_t *data, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)((0x10000 - sum) & 0xFFFF);
}

void send_read_command(int fd, uint8_t reg) {
    uint8_t cmd[] = {0xDD, 0xA5, reg, 0x00, 0x00, 0x00, 0x77};
    // Calculate checksum for payload bytes: Command(0xA5), Reg, Len(0x00)
    uint16_t cs = calculate_checksum(&cmd[1], 3);
    cmd[4] = (cs >> 8) & 0xFF;
    cmd[5] = cs & 0xFF;

    tcflush(fd, TCIFLUSH);
    write(fd, cmd, 7);
}

int read_response(int fd, uint8_t *buffer, int max_len) {
    int bytes_read = 0;
    int total_read = 0;

    // Read Header (4 bytes: Start, Reg, Status, Len)
    while (total_read < 4) {
        bytes_read = read(fd, buffer + total_read, 4 - total_read);
        if (bytes_read <= 0) return -1;
        total_read += bytes_read;
    }

    if (buffer[0] != 0xDD) return -2; // Bad start
    if (buffer[2] != 0x00) return -3; // BMS Error Status

    int data_len = buffer[3];
    int expected_total = 4 + data_len + 3; // Header(4) + Data(N) + CS(2) + End(1)

    if (expected_total > max_len) return -4; // Buffer overflow

    while (total_read < expected_total) {
        bytes_read = read(fd, buffer + total_read, expected_total - total_read);
        if (bytes_read <= 0) return -1;
        total_read += bytes_read;
    }

    if (buffer[expected_total - 1] != 0x77) return -5; // Bad end

    // Validate Checksum (Reg + Status + Len + DataBytes)
    uint16_t calc_cs = calculate_checksum(&buffer[1], data_len + 3);
    uint16_t recv_cs = (buffer[expected_total - 3] << 8) | buffer[expected_total - 2];

    if (calc_cs != recv_cs) {
        printf("Checksum mismatch! Calc: %04X, Recv: %04X\n", calc_cs, recv_cs);
        return -6;
    }

    return data_len; // Return length of actual payload
}

// --- Config Parser ---
void load_config(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        perror("Could not open config file");
        exit(1);
    }

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *key = strtok(line, "=");
        char *val = strtok(NULL, "\n");
        if (!key || !val) continue;

        if (strcmp(key, "serial_port") == 0) strcpy(config.serial_port, val);
        else if (strcmp(key, "baud_rate") == 0) config.baud_rate = atoi(val);
        else if (strcmp(key, "mqtt_broker") == 0) strcpy(config.mqtt_broker, val);
        else if (strcmp(key, "mqtt_port") == 0) config.mqtt_port = atoi(val);
        else if (strcmp(key, "mqtt_user") == 0) strcpy(config.mqtt_user, val);
        else if (strcmp(key, "mqtt_password") == 0) strcpy(config.mqtt_password, val);
        else if (strcmp(key, "poll_interval") == 0) config.poll_interval = atoi(val);
        else if (strcmp(key, "device_name") == 0) strcpy(config.device_name, val);
        else if (strcmp(key, "device_id") == 0) strcpy(config.device_id, val);
    }
    fclose(f);
}

// --- MQTT Discovery ---
void publish_sensor_config(struct mosquitto *mosq, const char *sensor_id, const char *name, const char *unit, const char *dev_class) {
    char topic[256];
    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", config.device_id, sensor_id);

    cJSON *root = cJSON_CreateObject();
    char full_name[128];
    snprintf(full_name, sizeof(full_name), "%s %s", config.device_name, name);
    cJSON_AddStringToObject(root, "name", full_name);

    char state_topic[128];
    snprintf(state_topic, sizeof(state_topic), "%s/status", config.device_id);
    cJSON_AddStringToObject(root, "state_topic", state_topic);

    char tmpl[64];
    snprintf(tmpl, sizeof(tmpl), "{{ value_json.%s }}", sensor_id);
    cJSON_AddStringToObject(root, "value_template", tmpl);

    if (unit) cJSON_AddStringToObject(root, "unit_of_measurement", unit);
    if (dev_class) cJSON_AddStringToObject(root, "device_class", dev_class);

    char unique_id[128];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", config.device_id, sensor_id);
    cJSON_AddStringToObject(root, "unique_id", unique_id);

    cJSON *device = cJSON_CreateObject();
    cJSON_AddStringToObject(device, "identifiers", config.device_id);
    cJSON_AddStringToObject(device, "name", config.device_name);
    cJSON_AddStringToObject(device, "manufacturer", "JBD");
    cJSON_AddStringToObject(device, "model", "RS485 BMS");
    cJSON_AddItemToObject(root, "device", device);

    char *json_str = cJSON_PrintUnformatted(root);
    mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, true);
    free(json_str);
    cJSON_Delete(root);
}

void send_discovery(struct mosquitto *mosq, int cell_count, int ntc_count) {
    publish_sensor_config(mosq, "pack_voltage", "Pack Voltage", "V", "voltage");
    publish_sensor_config(mosq, "pack_current", "Pack Current", "A", "current");
    publish_sensor_config(mosq, "soc", "SOC", "%", "battery");
    publish_sensor_config(mosq, "capacity_remain", "Capacity Remaining", "Ah", NULL);
    publish_sensor_config(mosq, "cycles", "Cycles", NULL, NULL);

    // Cell Voltages
    for (int i = 0; i < cell_count; i++) {
        char id[32], name[32];
        snprintf(id, sizeof(id), "cell_%d", i+1);
        snprintf(name, sizeof(name), "Cell %d", i+1);
        publish_sensor_config(mosq, id, name, "V", "voltage");
    }

    // Temperatures
    for (int i = 0; i < ntc_count; i++) {
        char id[32], name[32];
        snprintf(id, sizeof(id), "temp_%d", i+1);
        snprintf(name, sizeof(name), "Temp %d", i+1);
        publish_sensor_config(mosq, id, name, "°C", "temperature");
    }

    printf("Home Assistant Discovery Config Sent\n");
}

int main() {
    load_config("config.ini");
    printf("Starting JBD Exporter for %s on %s\n", config.device_name, config.serial_port);

    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(config.device_id, true, NULL);
    if (strlen(config.mqtt_user) > 0) {
        mosquitto_username_pw_set(mosq, config.mqtt_user, config.mqtt_password);
    }

    if (mosquitto_connect(mosq, config.mqtt_broker, config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to connect to MQTT broker\n");
        return 1;
    }
    mosquitto_loop_start(mosq);

    int serial_fd = setup_serial(config.serial_port);
    if (serial_fd < 0) return 1;

    uint8_t rx_buffer[256];
    int discovery_sent = 0;

    while (1) {
        cJSON *json = cJSON_CreateObject();

        // --- 1. Read Register 0x03 (Basic Info & Temps) ---
        send_read_command(serial_fd, 0x03);
        usleep(100000); // Wait 100ms
        int len = read_response(serial_fd, rx_buffer, sizeof(rx_buffer));

        int cell_count = 0;
        int ntc_count = 0;

        if (len > 0) {
            uint8_t *data = &rx_buffer[4]; // Skip header

            // Basic Info (Offsets based on glossary)
            uint16_t pack_mv = (data[0] << 8) | data[1];
            int16_t pack_ma = (int16_t)((data[2] << 8) | data[3]);
            uint16_t cap_rem = (data[4] << 8) | data[5];
            uint16_t cycles = (data[8] << 8) | data[9];
            uint16_t protect = (data[16] << 8) | data[17];
            uint8_t soc = data[19];
            uint8_t fet_status = data[20];

            cell_count = data[21];
            ntc_count = data[22];

            cJSON_AddNumberToObject(json, "pack_voltage", pack_mv / 100.0);
            cJSON_AddNumberToObject(json, "pack_current", pack_ma / 100.0);
            cJSON_AddNumberToObject(json, "capacity_remain", cap_rem / 100.0);
            cJSON_AddNumberToObject(json, "cycles", cycles);
            cJSON_AddNumberToObject(json, "soc", soc);
            cJSON_AddNumberToObject(json, "protection_bits", protect);
            cJSON_AddBoolToObject(json, "mosfet_charge", fet_status & 1);
            cJSON_AddBoolToObject(json, "mosfet_discharge", fet_status & 2);

            // Parse Temperatures
            // NTC values start at byte 23. Each is 2 bytes.
            // Unit 0.1K. C = (K * 0.1) - 273.15
            int ntc_offset = 23;
            for (int i = 0; i < ntc_count; i++) {
                if (ntc_offset + 1 < len) {
                    uint16_t raw_temp = (data[ntc_offset] << 8) | data[ntc_offset+1];
                    double temp_c = (raw_temp * 0.1) - 273.15;
                    char key[16];
                    snprintf(key, sizeof(key), "temp_%d", i+1);
                    cJSON_AddNumberToObject(json, key, temp_c);
                    ntc_offset += 2;
                }
            }

            // --- 2. Read Register 0x04 (Cell Voltages) ---
            if (cell_count > 0) {
                send_read_command(serial_fd, 0x04);
                usleep(100000);
                int len_cells = read_response(serial_fd, rx_buffer, sizeof(rx_buffer));

                if (len_cells > 0) {
                    uint8_t *cdata = &rx_buffer[4];
                    for (int i = 0; i < cell_count; i++) {
                        if (i*2 + 1 < len_cells) {
                            uint16_t cmv = (cdata[i*2] << 8) | cdata[i*2+1];
                            char key[16];
                            snprintf(key, sizeof(key), "cell_%d", i+1);
                            cJSON_AddNumberToObject(json, key, cmv / 1000.0);
                        }
                    }
                }
            }

            // Publish Status
            char *json_str = cJSON_PrintUnformatted(json);
            char topic[128];
            snprintf(topic, sizeof(topic), "%s/status", config.device_id);
            mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, false);
            printf("Published Data (SOC: %d%%)\n", soc);
            free(json_str);

            // Send Discovery Once (now that we know the counts)
            if (!discovery_sent && cell_count > 0) {
                send_discovery(mosq, cell_count, ntc_count);
                discovery_sent = 1;
            }
        } else {
            printf("Failed to read BMS Reg 0x03 (Error Code: %d)\n", len);
        }

        cJSON_Delete(json);
        sleep(config.poll_interval);
    }

    close(serial_fd);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
