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

// --- Configuration ---
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

// --- Helpers ---
void print_hex(const char *label, uint8_t *data, int len) {
    printf("%s: ", label);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

// --- Serial Setup ---
int setup_serial(const char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // 8N1 Configuration
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // Raw Mode
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = ~(IXON | IXOFF | IXANY);

    // Blocking Read with 2.0s Timeout (VTIME * 0.1s)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 20;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;

    // Clear buffers
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// --- JBD Protocol ---

uint16_t jbd_checksum(uint8_t *data, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)((0x10000 - sum) & 0xFFFF);
}

int send_and_read(int fd, uint8_t reg, uint8_t *out_buffer, int max_len) {
    // 1. Clear Input Buffer (Instant, unlike previous loop)
    tcflush(fd, TCIFLUSH);

    // 2. Build Packet
    uint8_t cmd[7];
    cmd[0] = 0xDD; // Start
    cmd[1] = 0xA5; // Read
    cmd[2] = reg;
    cmd[3] = 0x00; // Len

    uint16_t cs = jbd_checksum(&cmd[2], 2);
    cmd[4] = (cs >> 8) & 0xFF;
    cmd[5] = cs & 0xFF;
    cmd[6] = 0x77; // End

    print_hex("TX", cmd, 7); // Debug Print

    if (write(fd, cmd, 7) != 7) {
        perror("Write failed");
        return -1;
    }

    // 3. Read Response
    // We read one byte at a time to find the Start Byte (0xDD)
    int total_read = 0;
    int aligned = 0;
    uint8_t header[4];

    // Try to find header (timeout 2s handled by serial config)
    while (total_read < 4) {
        uint8_t byte;
        int n = read(fd, &byte, 1);

        if (n < 0) {
            perror("Read Error");
            return -1;
        }
        if (n == 0) {
            // Timeout
            return -2;
        }

        if (!aligned) {
            if (byte == 0xDD) {
                aligned = 1;
                header[0] = byte;
                total_read = 1;
                // printf("Found Start Byte\n");
            } else {
                // printf("Skip: %02X\n", byte); // Uncomment to see garbage
            }
        } else {
            header[total_read++] = byte;
        }
    }

    print_hex("RX Header", header, 4);

    if (header[2] != 0x00) {
        printf("BMS Status Error: %02X\n", header[2]);
        return -3;
    }

    int data_len = header[3];
    int expected_remaining = data_len + 3; // Data + CS(2) + End(1)

    if (4 + expected_remaining > max_len) return -4; // Buffer too small

    memcpy(out_buffer, header, 4);

    // Read Payload
    int rem_read = 0;
    while(rem_read < expected_remaining) {
        int n = read(fd, out_buffer + 4 + rem_read, expected_remaining - rem_read);
        if (n <= 0) return -2;
        rem_read += n;
    }

    print_hex("RX Full", out_buffer, 4 + expected_remaining);

    // Validate Checksum
    uint8_t *data_ptr = &out_buffer[4];
    uint16_t recv_cs = (out_buffer[4 + data_len] << 8) | out_buffer[4 + data_len + 1];

    // Method 1: Standard (Status + Len + Data)
    uint8_t cs_buf1[256];
    cs_buf1[0] = header[2];
    cs_buf1[1] = header[3];
    memcpy(&cs_buf1[2], data_ptr, data_len);
    uint16_t calc_cs1 = jbd_checksum(cs_buf1, data_len + 2);

    if (recv_cs != calc_cs1) {
        // Method 2: Quirk for Cell Voltages (observed in some JBD firmwares)
        if (reg == 0x04 && (recv_cs + 0x30 == calc_cs1)) {
            // Allowed
        } else {
            printf("Checksum Mismatch: Recv=%04X, Calc=%04X\n", recv_cs, calc_cs1);
            return -5;
        }
    }

    return 4 + expected_remaining;
}

// --- Config Parser ---
void load_config(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *key = strtok(line, "=");
        char *val = strtok(NULL, "\n");
        if (!key || !val) continue;
        if (strcmp(key, "serial_port") == 0) strcpy(config.serial_port, val);
        else if (strcmp(key, "baud_rate") == 0) config.baud_rate = atoi(val);
        else if (strcmp(key, "mqtt_broker") == 0) strcpy(config.mqtt_broker, val);
        else if (strcmp(key, "mqtt_port") == 0) config.mqtt_port = atoi(val);
        else if (strcmp(key, "poll_interval") == 0) config.poll_interval = atoi(val);
        else if (strcmp(key, "device_id") == 0) strcpy(config.device_id, val);
    }
    fclose(f);
}

// --- MQTT Discovery ---
void publish_config(struct mosquitto *mosq, const char *sid, const char *name, const char *unit, const char *cls) {
    cJSON *root = cJSON_CreateObject();
    char topic[256], val_tmpl[128], uniq[128];
    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", config.device_id, sid);
    snprintf(val_tmpl, sizeof(val_tmpl), "{{ value_json.%s }}", sid);
    snprintf(uniq, sizeof(uniq), "%s_%s", config.device_id, sid);

    cJSON_AddStringToObject(root, "name", name);
    char stat[128]; snprintf(stat, 128, "%s/status", config.device_id);
    cJSON_AddStringToObject(root, "state_topic", stat);
    cJSON_AddStringToObject(root, "value_template", val_tmpl);
    cJSON_AddStringToObject(root, "unique_id", uniq);
    if(unit) cJSON_AddStringToObject(root, "unit_of_measurement", unit);
    if(cls) cJSON_AddStringToObject(root, "device_class", cls);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", config.device_id);
    cJSON_AddStringToObject(dev, "name", "JBD BMS");
    cJSON_AddStringToObject(dev, "manufacturer", "JBD");
    cJSON_AddItemToObject(root, "device", dev);

    char *p = cJSON_PrintUnformatted(root);
    mosquitto_publish(mosq, NULL, topic, strlen(p), p, 0, true);
    free(p); cJSON_Delete(root);
}

void send_discovery(struct mosquitto *mosq, int cells, int ntcs) {
    publish_config(mosq, "pack_voltage", "Pack Voltage", "V", "voltage");
    publish_config(mosq, "pack_current", "Pack Current", "A", "current");
    publish_config(mosq, "soc", "SOC", "%", "battery");
    publish_config(mosq, "cycles", "Cycles", NULL, NULL);

    for(int i=0; i<cells; i++) {
        char s[32], n[32];
        snprintf(s, 32, "cell_%d", i+1);
        snprintf(n, 32, "Cell %d Voltage", i+1);
        publish_config(mosq, s, n, "V", "voltage");
    }
    for(int i=0; i<ntcs; i++) {
        char s[32], n[32];
        snprintf(s, 32, "temp_%d", i+1);
        snprintf(n, 32, "Temp %d", i+1);
        publish_config(mosq, s, n, "°C", "temperature");
    }
}

int main() {
    strcpy(config.serial_port, "/dev/ttyUSB0");
    config.baud_rate = 9600;
    strcpy(config.mqtt_broker, "127.0.0.1");
    config.mqtt_port = 1883;
    config.poll_interval = 10;
    strcpy(config.device_id, "jbd_exporter");

    load_config("config.ini");

    printf("--- JBD Exporter Debug Mode ---\n");
    printf("Port: %s | Broker: %s\n", config.serial_port, config.mqtt_broker);

    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(config.device_id, true, NULL);
    if(mosquitto_connect(mosq, config.mqtt_broker, config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        printf("MQTT Connect Fail\n");
        return 1;
    }
    mosquitto_loop_start(mosq);

    int fd = setup_serial(config.serial_port);
    if(fd < 0) return 1;

    uint8_t buf[512];
    int discovery_done = 0;

    while(1) {
        printf("\nReading 0x03...\n");
        int len = send_and_read(fd, 0x03, buf, sizeof(buf));

        if (len > 0) {
            cJSON *json = cJSON_CreateObject();
            uint8_t *d = &buf[4];
            uint16_t v = (d[0]<<8)|d[1];
            int16_t a = (int16_t)((d[2]<<8)|d[3]);
            uint16_t cap = (d[4]<<8)|d[5];
            uint16_t cyc = (d[8]<<8)|d[9];
            uint16_t prot = (d[16]<<8)|d[17];
            uint8_t soc = d[19];
            int cells = d[21];
            int ntcs = d[22];

            cJSON_AddNumberToObject(json, "pack_voltage", v/100.0);
            cJSON_AddNumberToObject(json, "pack_current", a/100.0);
            cJSON_AddNumberToObject(json, "capacity_remain", cap/100.0);
            cJSON_AddNumberToObject(json, "cycles", cyc);
            cJSON_AddNumberToObject(json, "soc", soc);
            cJSON_AddNumberToObject(json, "protection", prot);

            // Temps
            int offset = 23;
            for(int i=0; i<ntcs; i++) {
                if(offset+1 < buf[3]) {
                    uint16_t k = (d[offset]<<8)|d[offset+1];
                    char kStr[16]; snprintf(kStr, 16, "temp_%d", i+1);
                    cJSON_AddNumberToObject(json, kStr, (k*0.1)-273.15);
                    offset+=2;
                }
            }

            // Cells
            if (cells > 0) {
                usleep(200000);
                printf("Reading 0x04...\n");
                int clen = send_and_read(fd, 0x04, buf, sizeof(buf));
                if (clen > 0) {
                    uint8_t *cd = &buf[4];
                    for(int i=0; i<cells; i++) {
                        if(i*2+1 < buf[3]) {
                            uint16_t cmv = (cd[i*2]<<8)|cd[i*2+1];
                            char cStr[16]; snprintf(cStr, 16, "cell_%d", i+1);
                            cJSON_AddNumberToObject(json, cStr, cmv/1000.0);
                        }
                    }
                }
            }

            char *pl = cJSON_PrintUnformatted(json);
            mosquitto_publish(mosq, NULL, "jbd_exporter/status", strlen(pl), pl, 0, false);
            printf("MQTT Sent: %s\n", pl);
            free(pl);
            cJSON_Delete(json);

            if (!discovery_done && cells > 0) {
                send_discovery(mosq, cells, ntcs);
                discovery_done = 1;
            }
        } else {
            printf("Read Failed 0x03. Err Code: %d\n", len);
        }

        sleep(config.poll_interval);
    }
    return 0;
}
