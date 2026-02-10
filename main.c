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

// --- Helper: Hex Dump for Debugging ---
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

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // Raw mode
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = ~(IXON | IXOFF | IXANY);

    // Timeout: Important for JBD
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // 1.0 second timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;

    // Clear any garbage currently in buffer
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// --- JBD Protocol Implementation (Based on your Reference) ---

// Standard JBD Checksum: 0x10000 - sum
uint16_t jbd_checksum(uint8_t *data, int len) {
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)((0x10000 - sum) & 0xFFFF);
}

// Flush input buffer manually (robustness fix)
void flush_input(int fd) {
    uint8_t garbage[128];
    while(read(fd, garbage, sizeof(garbage)) > 0);
}

int send_and_read(int fd, uint8_t reg, uint8_t *out_buffer, int max_len) {
    // 1. Flush Input Buffer (Crucial for RS485 echoes)
    flush_input(fd);

    // 2. Build Packet
    // Protocol: Start(DD) | Read(A5) | Reg | Len(0) | CS_H | CS_L | End(77)
    uint8_t cmd[7];
    cmd[0] = 0xDD;
    cmd[1] = 0xA5;
    cmd[2] = reg;
    cmd[3] = 0x00;

    // Calculate Checksum on [Reg, Len] only, matching your reference code
    // (Some firmwares vary, but your reference checks specific bytes)
    uint16_t cs = jbd_checksum(&cmd[2], 2);

    cmd[4] = (cs >> 8) & 0xFF;
    cmd[5] = cs & 0xFF;
    cmd[6] = 0x77;

    // 3. Send
    // printf("Sending CMD: %02X\n", reg); // Uncomment to debug TX
    if (write(fd, cmd, 7) != 7) {
        perror("Write failed");
        return -1;
    }

    // 4. Wait slightly (Reference uses 100ms)
    usleep(100000);

    // 5. Read Response - Robust Loop
    // We look for 0xDD first to align the frame
    uint8_t byte;
    int bytes_read = 0;
    int aligned = 0;
    int total_read = 0;

    // Read Header (4 bytes)
    uint8_t header[4];
    while(total_read < 4) {
        int n = read(fd, &byte, 1);
        if (n <= 0) return -2; // Timeout

        if (!aligned) {
            if (byte == 0xDD) {
                aligned = 1;
                header[0] = byte;
                total_read = 1;
            }
        } else {
            header[total_read++] = byte;
        }
    }

    // Header: [DD] [Reg] [Status] [Len]
    if (header[2] != 0x00) {
        printf("BMS Error Status: %02X\n", header[2]);
        return -3;
    }

    int data_len = header[3];
    int expected_remaining = data_len + 3; // Data + CS(2) + End(1)

    if (4 + expected_remaining > max_len) return -4; // Buffer too small

    // Copy header to out_buffer
    memcpy(out_buffer, header, 4);

    // Read remaining bytes
    int rem_read = 0;
    while(rem_read < expected_remaining) {
        int n = read(fd, out_buffer + 4 + rem_read, expected_remaining - rem_read);
        if (n <= 0) return -2;
        rem_read += n;
    }

    // 6. Validate Checksum (Permissive Mode from Reference)
    uint8_t *data_ptr = &out_buffer[4];
    uint16_t recv_cs = (out_buffer[4 + data_len] << 8) | out_buffer[4 + data_len + 1];

    // Checksum 1: Status + Len + Data (Standard)
    // Construct buffer for calc: [Status, Len, Data...]
    uint8_t cs_buf1[256];
    cs_buf1[0] = header[2]; // Status
    cs_buf1[1] = header[3]; // Len
    memcpy(&cs_buf1[2], data_ptr, data_len);
    uint16_t calc_cs1 = jbd_checksum(cs_buf1, data_len + 2);

    // Checksum 2: Cmd + Status + Len + Data (Variant)
    uint8_t cs_buf2[256];
    cs_buf2[0] = header[1]; // Cmd (Reg)
    cs_buf2[1] = header[2]; // Status
    cs_buf2[2] = header[3]; // Len
    memcpy(&cs_buf2[3], data_ptr, data_len);
    uint16_t calc_cs2 = jbd_checksum(cs_buf2, data_len + 3);

    int valid = 0;
    if (recv_cs == calc_cs1) valid = 1;
    else if (recv_cs == calc_cs2) valid = 1;
    // Quirk for 0x04 from your reference
    else if (reg == 0x04 && (recv_cs + 0x30 == calc_cs1)) valid = 1;

    if (!valid) {
        printf("CS Fail. Recv: %04X, Calc1: %04X, Calc2: %04X\n", recv_cs, calc_cs1, calc_cs2);
        print_hex("Raw Packet", out_buffer, 4 + expected_remaining);
        return -5;
    }

    return 4 + expected_remaining; // Return total frame length
}

// --- Config Parser ---
void load_config(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
        printf("Config file not found, using defaults.\n");
        return;
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
void publish_config(struct mosquitto *mosq, const char *sid, const char *name, const char *unit, const char *cls) {
    cJSON *root = cJSON_CreateObject();
    char full_name[128], topic[256], uniq[128], state[128], tmpl[64];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", config.device_id, sid);
    snprintf(full_name, sizeof(full_name), "%s %s", config.device_name, name);
    snprintf(uniq, sizeof(uniq), "%s_%s", config.device_id, sid);
    snprintf(state, sizeof(state), "%s/status", config.device_id);
    snprintf(tmpl, sizeof(tmpl), "{{ value_json.%s }}", sid);

    cJSON_AddStringToObject(root, "name", full_name);
    cJSON_AddStringToObject(root, "state_topic", state);
    cJSON_AddStringToObject(root, "value_template", tmpl);
    cJSON_AddStringToObject(root, "unique_id", uniq);
    if(unit) cJSON_AddStringToObject(root, "unit_of_measurement", unit);
    if(cls) cJSON_AddStringToObject(root, "device_class", cls);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", config.device_id);
    cJSON_AddStringToObject(dev, "name", config.device_name);
    cJSON_AddStringToObject(dev, "manufacturer", "JBD");
    cJSON_AddItemToObject(root, "device", dev);

    char *payload = cJSON_PrintUnformatted(root);
    mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, true);
    free(payload);
    cJSON_Delete(root);
}

void send_discovery(struct mosquitto *mosq, int cells, int ntcs) {
    publish_config(mosq, "pack_voltage", "Pack Voltage", "V", "voltage");
    publish_config(mosq, "pack_current", "Pack Current", "A", "current");
    publish_config(mosq, "soc", "SOC", "%", "battery");
    publish_config(mosq, "capacity_remain", "Capacity Remaining", "Ah", NULL);
    publish_config(mosq, "cycles", "Cycles", NULL, NULL);

    for(int i=0; i<cells; i++) {
        char s[32], n[32];
        snprintf(s, 32, "cell_%d", i+1);
        snprintf(n, 32, "Cell %d", i+1);
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
    // Defaults
    strcpy(config.serial_port, "/dev/ttyUSB0");
    config.baud_rate = 9600;
    strcpy(config.mqtt_broker, "127.0.0.1");
    config.mqtt_port = 1883;
    config.poll_interval = 10;
    strcpy(config.device_name, "JBD BMS");
    strcpy(config.device_id, "jbd_exporter");

    load_config("config.ini");

    printf("--- JBD Exporter Starting ---\n");
    printf("Port: %s | Broker: %s\n", config.serial_port, config.mqtt_broker);

    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(config.device_id, true, NULL);
    if(strlen(config.mqtt_user)>0) mosquitto_username_pw_set(mosq, config.mqtt_user, config.mqtt_password);

    if(mosquitto_connect(mosq, config.mqtt_broker, config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        printf("MQTT Connection Failed\n");
        return 1;
    }
    mosquitto_loop_start(mosq);

    int fd = setup_serial(config.serial_port);
    if(fd < 0) return 1;

    uint8_t buf[256];
    int discovery_done = 0;

    while(1) {
        cJSON *json = cJSON_CreateObject();

        // 1. Read Basic Info (0x03)
        int len = send_and_read(fd, 0x03, buf, sizeof(buf));
        int cells = 0, ntcs = 0;

        if(len > 0) {
            uint8_t *d = &buf[4]; // Data start
            uint16_t volt = (d[0]<<8)|d[1];
            int16_t amps = (int16_t)((d[2]<<8)|d[3]);
            uint16_t cap = (d[4]<<8)|d[5];
            uint16_t cyc = (d[8]<<8)|d[9];
            uint16_t prot = (d[16]<<8)|d[17];
            uint8_t soc = d[19];
            cells = d[21];
            ntcs = d[22];

            cJSON_AddNumberToObject(json, "pack_voltage", volt/100.0);
            cJSON_AddNumberToObject(json, "pack_current", amps/100.0);
            cJSON_AddNumberToObject(json, "capacity_remain", cap/100.0);
            cJSON_AddNumberToObject(json, "cycles", cyc);
            cJSON_AddNumberToObject(json, "soc", soc);
            cJSON_AddNumberToObject(json, "protection", prot);

            // Temps (Start at byte 23, 2 bytes each)
            int offset = 23;
            for(int i=0; i<ntcs; i++) {
                if(offset+1 < buf[3]) { // check against data len
                    uint16_t k = (d[offset]<<8)|d[offset+1];
                    char kStr[16]; snprintf(kStr, 16, "temp_%d", i+1);
                    cJSON_AddNumberToObject(json, kStr, (k*0.1)-273.15);
                    offset+=2;
                }
            }

            // 2. Read Cells (0x04)
            if(cells > 0) {
                // Short delay between commands
                usleep(150000);
                int clen = send_and_read(fd, 0x04, buf, sizeof(buf));
                if(clen > 0) {
                    uint8_t *cd = &buf[4];
                    for(int i=0; i<cells; i++) {
                        if(i*2+1 < buf[3]) {
                            uint16_t mv = (cd[i*2]<<8)|cd[i*2+1];
                            char cStr[16]; snprintf(cStr, 16, "cell_%d", i+1);
                            cJSON_AddNumberToObject(json, cStr, mv/1000.0);
                        }
                    }
                }
            }

            char *pl = cJSON_PrintUnformatted(json);
            char top[128]; snprintf(top, 128, "%s/status", config.device_id);
            mosquitto_publish(mosq, NULL, top, strlen(pl), pl, 0, false);
            printf("Data Sent: SOC %d%%\n", soc);
            free(pl);

            if(!discovery_done && cells > 0) {
                send_discovery(mosq, cells, ntcs);
                discovery_done = 1;
            }
        } else {
            printf("Read Failed (Err: %d)\n", len);
        }

        cJSON_Delete(json);
        sleep(config.poll_interval);
    }

    close(fd);
    return 0;
}
